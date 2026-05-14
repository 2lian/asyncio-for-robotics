import asyncio
import logging
import threading
from asyncio import AbstractEventLoop
from typing import AsyncGenerator, Generic, Optional, TypeVar, cast

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient as RosActionClient
from rclpy.action import ActionServer as RosActionServer
from rclpy.action.client import ClientGoalHandle as RosClientGoalHandle

from ..core.scope import AUTO_SCOPE, Scope
from ..core.sub import BaseSub
from .future import asyncify_future
from .session import BaseSession, auto_session

logger = logging.getLogger(__name__)

_GoalT = TypeVar("_GoalT")
_FeedbackT = TypeVar("_FeedbackT")
_ResultT = TypeVar("_ResultT")


# ── Exceptions ────────────────────────────────────────────────────────────────


class ActionAborted(Exception):
    """Raised when awaiting ``goal_handle.result`` after the server aborts the goal.

    Attributes:
        result: The result object returned by the server at abort time.
            Its fields are action-type specific; may contain partial data.

    Example::

        try:
            result = await goal_handle.result
        except ActionAborted as e:
            handle_abort(e.result)
    """

    def __init__(self, result) -> None:
        self.result = result
        super().__init__(repr(result))


class ActionCanceled(Exception):
    """Raised when awaiting ``goal_handle.result`` after the goal is canceled.

    Cancellation requires the server to (1) register a ``cancel_callback`` that
    returns ``CancelResponse.ACCEPT`` and (2) detect ``goal_handle.is_cancel_requested``
    in its execute loop and call ``goal_handle.canceled(result)``.

    Attributes:
        result: The result object returned by the server at cancel time.

    Example::

        try:
            result = await goal_handle.result
        except ActionCanceled as e:
            handle_cancel(e.result)
    """

    def __init__(self, result) -> None:
        self.result = result
        super().__init__(repr(result))


# ── Server-side goal handle ───────────────────────────────────────────────────


class ActionGoalHandle(Generic[_GoalT, _FeedbackT, _ResultT]):
    """Server-side goal handle delivered to user asyncio code by ``ActionServer``.

    Bridges the rclpy executor thread (where the ROS 2 execute callback runs) and
    the asyncio event loop (where user code runs).  Instances are created by
    ``ActionServer._execute`` and pushed to the asyncio queue; the executor thread
    blocks until the user calls ``succeed()``, ``abort()``, or ``canceled()``.

    Delivered via ``ActionServer.listen_reliable()`` (or any other ``BaseSub``
    listening method).  Each handle represents exactly one in-flight goal.

    Attributes:
        request: The goal request message sent by the client (read-only).
        is_cancel_requested: True once the client has requested cancellation and
            the server's ``cancel_callback`` accepted it.

    Example::

        async for goal_handle in server.listen_reliable():
            # Dispatch concurrently so the server can accept new goals
            asyncio.create_task(handle_goal(goal_handle))

        async def handle_goal(goal_handle):
            fb = MyAction.Feedback()
            for step in compute_steps(goal_handle.request):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled(MyAction.Result())
                    return
                fb.progress = step
                goal_handle.publish_feedback(fb)
                await asyncio.sleep(0)
            goal_handle.succeed(MyAction.Result(value=42))

    .. Important::
        ``succeed()`` / ``abort()`` / ``canceled()`` must be called **exactly once**.
        Failing to call any of them leaves the executor thread blocked indefinitely,
        preventing new goals from being dispatched on that thread.
    """

    def __init__(self, ros_gh, event_loop: AbstractEventLoop) -> None:
        self._ros_gh = ros_gh
        self._event_loop = event_loop
        self._done = threading.Event()
        self._result: Optional[_ResultT] = None

    @property
    def request(self) -> _GoalT:
        """The goal message sent by the client (e.g. ``Fibonacci.Goal``)."""
        return self._ros_gh.request

    @property
    def is_cancel_requested(self) -> bool:
        """True when the client has requested cancellation and the server accepted it.

        Poll this inside the execute loop and call ``canceled()`` when True.
        The flag is set by the rclpy action server in a separate executor thread;
        reading it from the asyncio loop is safe.

        .. Note::
            This flag is only set if the server was created with a ``cancel_callback``
            that returns ``CancelResponse.ACCEPT``.  rclpy's default rejects all
            cancel requests, so the flag will never be True without an explicit callback.
        """
        return self._ros_gh.is_cancel_requested

    def publish_feedback(self, feedback: _FeedbackT) -> None:
        """Publish an intermediate feedback message to the client.

        May be called from any thread (the underlying rclpy call is thread-safe).

        Args:
            feedback: Populated feedback message (e.g. ``Fibonacci.Feedback``).
        """
        self._ros_gh.publish_feedback(feedback)

    def succeed(self, result: _ResultT) -> None:
        """Finish the goal with SUCCESS and deliver the result to the client.

        Unblocks the executor thread.  Call exactly once per goal handle.

        Args:
            result: Populated result message (e.g. ``Fibonacci.Result``).
        """
        self._result = result
        self._ros_gh.succeed()
        self._done.set()

    def abort(self, result: _ResultT) -> None:
        """Finish the goal with ABORTED and deliver the result to the client.

        Awaiting the client's ``result`` future will raise ``ActionAborted(result)``.
        Unblocks the executor thread.  Call exactly once per goal handle.

        Args:
            result: Populated result message describing what was done before aborting.
        """
        self._result = result
        self._ros_gh.abort()
        self._done.set()

    def canceled(self, result: _ResultT) -> None:
        """Finish the goal with CANCELED and deliver the result to the client.

        Call this after detecting ``is_cancel_requested == True``.
        Awaiting the client's ``result`` future will raise ``ActionCanceled(result)``.
        Unblocks the executor thread.  Call exactly once per goal handle.

        Args:
            result: Populated result message with any partial output collected so far.
        """
        self._result = result
        self._ros_gh.canceled()
        self._done.set()

    def _wait_for_completion(self) -> _ResultT:
        """Block the executor thread until the user finishes handling this goal.

        Called internally by ActionServer._execute.  Never call this yourself.
        """
        self._done.wait()
        return self._result  # type: ignore[return-value]


# ── Client-side goal handle ───────────────────────────────────────────────────


class ActionFeedbackDone:
    """Marker yielded by action feedback streams when the result has arrived."""

    def __repr__(self) -> str:
        return "ActionFeedbackDone()"


class ActionFeedbackSub(BaseSub[_FeedbackT | ActionFeedbackDone], Generic[_FeedbackT]):
    """Feedback subscriber that keeps the result sentinel in order."""

    def listen(self, fresh=False):
        return self.listen_reliable(fresh=fresh, queue_size=0)

    @property
    def name(self) -> str:
        return "ROS2-ACT-FEEDBACK"


class ClientGoalHandle(Generic[_GoalT, _FeedbackT, _ResultT]):
    """Async wrapper around rclpy.action.client.ClientGoalHandle.

    Returned by ``ActionClient.send_goal()``.

    ``accepted`` and ``result`` are asyncio futures.  ``feedback`` is a normal
    afor subscriber that receives feedback messages and an ``ActionFeedbackDone``
    value when the result arrives.

    Example::

        gh = client.send_goal(goal)
        accepted = await gh.accepted
        async for feedback in gh.feedback_until_result():
            process(feedback)
        result = await gh.result
    """

    SENTINEL = ActionFeedbackDone()

    def __init__(
        self,
        event_loop: AbstractEventLoop,
        *,
        scope: Optional[Scope] = AUTO_SCOPE,
    ) -> None:
        self._ros_gh: Optional[RosClientGoalHandle] = None
        self._event_loop = event_loop
        self.accepted: asyncio.Future[bool] = event_loop.create_future()
        self.result: asyncio.Future[_ResultT] = event_loop.create_future()
        self.result.add_done_callback(self._consume_unhandled_result_exception)
        self.feedback: ActionFeedbackSub[_FeedbackT] = ActionFeedbackSub(scope=scope)

    def _watch_goal_response(self, ros_gh_future: asyncio.Future) -> None:
        ros_gh_future.add_done_callback(self._on_goal_response)

    @staticmethod
    def _consume_unhandled_result_exception(fut: asyncio.Future) -> None:
        if fut.cancelled():
            return
        fut.exception()

    def _on_goal_response(self, fut: asyncio.Future) -> None:
        if fut.cancelled():
            self.accepted.cancel()
            self.result.cancel()
            self.feedback.close()
            return
        exc = fut.exception()
        if exc is not None:
            self.accepted.set_exception(exc)
            self.result.set_exception(exc)
            self.feedback.close()
            return

        ros_gh: RosClientGoalHandle = fut.result()
        self._ros_gh = ros_gh
        self.accepted.set_result(ros_gh.accepted)
        if not ros_gh.accepted:
            self.result.set_exception(RuntimeError("goal was rejected by server"))
            self.feedback.input_data(self.SENTINEL)
            return

        result_future = asyncify_future(ros_gh.get_result_async(), self._event_loop)
        result_future.add_done_callback(self._on_result)

    def _on_result(self, fut: asyncio.Future) -> None:
        self.feedback.input_data(self.SENTINEL)
        if fut.cancelled():
            self.result.cancel()
            return
        exc = fut.exception()
        if exc is not None:
            self.result.set_exception(exc)
            return

        ros_result = fut.result()
        status = ros_result.status
        result = ros_result.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.result.set_result(result)
        elif status == GoalStatus.STATUS_ABORTED:
            self.result.set_exception(ActionAborted(result))
        elif status == GoalStatus.STATUS_CANCELED:
            self.result.set_exception(ActionCanceled(result))
        else:
            self.result.set_result(result)

    async def get_result(self) -> _ResultT:
        """Backward-compatible alias for awaiting ``result``."""
        return await self.result

    async def feedback_until_result(self) -> AsyncGenerator[_FeedbackT, None]:
        """Yield feedback messages until the action result arrives."""
        async for msg in self.feedback.listen():
            if isinstance(msg, ActionFeedbackDone):
                break
            yield cast(_FeedbackT, msg)

    async def cancel_goal(self) -> object:
        """Request cancellation of this goal and await the server's acknowledgment.

        Returns the ``CancelGoal.Response`` from rclpy.  Check
        ``response.goals_canceling`` to confirm the server accepted the request —
        an empty list means the server rejected the cancel (its ``cancel_callback``
        returned ``CancelResponse.REJECT``, or the goal had already terminated).

        .. Important::
            The server must be constructed with a ``cancel_callback`` that returns
            ``CancelResponse.ACCEPT``; rclpy's **default** rejects all cancel
            requests.  Without this, ``goals_canceling`` will always be empty and
            ``is_cancel_requested`` will never be set on the server side.
        """
        await self.accepted
        if self._ros_gh is None:
            raise RuntimeError("goal handle is not available")
        return await asyncify_future(self._ros_gh.cancel_goal_async(), self._event_loop)


# ── ActionServer ──────────────────────────────────────────────────────────────


class ActionServer(BaseSub["ActionGoalHandle"]):
    """Async ROS 2 action server implemented as an afor subscriber.

    Each accepted goal is delivered as an ``ActionGoalHandle`` through the
    standard ``BaseSub`` interface (``listen_reliable()``, ``wait_for_next()``,
    etc.).  The user must call exactly one of ``succeed`` / ``abort`` /
    ``canceled`` on the handle to complete the goal.

    .. Important::
        Requires the session's executor to be a ``MultiThreadedExecutor``
        (or equivalent).  The execute callback blocks an executor thread for the
        duration of each goal; a ``SingleThreadedExecutor`` will deadlock on the
        first goal.

    Args:
        action_type: The action interface class (e.g. ``Fibonacci``).
        action_name: ROS 2 action name (e.g. ``"fibonacci"``).
        session: ROS session to use.  Defaults to the current lexical session.
        goal_callback: Optional callable ``(goal_request) -> GoalResponse`` invoked
            before a goal is accepted.  Return ``GoalResponse.ACCEPT`` to accept or
            ``GoalResponse.REJECT`` to reject.  Defaults to rclpy's built-in
            which accepts all goals.
        cancel_callback: Optional callable ``(goal_handle) -> CancelResponse``
            invoked when the client requests cancellation.  Return
            ``CancelResponse.ACCEPT`` to honour the request or
            ``CancelResponse.REJECT`` to deny it.

            .. Warning::
                rclpy's **default** ``cancel_callback`` **rejects every cancel
                request**.  If you need clients to be able to cancel goals, you
                must pass ``cancel_callback=lambda _: CancelResponse.ACCEPT``
                (or a custom function).  Without this, ``is_cancel_requested``
                will never become ``True`` on the server side.

        scope: afor Scope for lifecycle management.

    Example — concurrent goals with cancel support::

        from rclpy.action import CancelResponse

        server = ActionServer(
            Fibonacci, "fibonacci",
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        async for goal_handle in server.listen_reliable():
            asyncio.create_task(handle_goal(goal_handle))

        async def handle_goal(goal_handle):
            seq = [0, 1]
            fb = Fibonacci.Feedback()
            for _ in range(1, goal_handle.request.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled(Fibonacci.Result(sequence=seq))
                    return
                seq.append(seq[-1] + seq[-2])
                fb.sequence = seq
                goal_handle.publish_feedback(fb)
                await asyncio.sleep(0.05)
            goal_handle.succeed(Fibonacci.Result(sequence=seq))
    """

    def __init__(
        self,
        action_type,
        action_name: str,
        session: Optional[BaseSession] = None,
        *,
        goal_callback=None,
        cancel_callback=None,
        scope: Optional[Scope] = AUTO_SCOPE,
    ) -> None:
        self._session: BaseSession = auto_session(session)
        self._action_type = action_type
        self._action_name = action_name
        self._goal_callback = goal_callback
        self._cancel_callback = cancel_callback
        self._ros_server: RosActionServer = self._create_ros_server()
        super().__init__(scope=scope)

    def _create_ros_server(self) -> RosActionServer:
        with self._session.lock() as node:
            kwargs: dict = {"execute_callback": self._execute}
            if self._goal_callback is not None:
                kwargs["goal_callback"] = self._goal_callback
            if self._cancel_callback is not None:
                kwargs["cancel_callback"] = self._cancel_callback
            server = RosActionServer(
                node, self._action_type, self._action_name, **kwargs
            )
        return server

    def _execute(self, ros_gh) -> object:
        """Execute callback (runs in executor thread).

        Creates an ActionGoalHandle, pushes it to the asyncio event loop for
        user handling, then blocks until the user resolves the goal.
        """
        goal_handle: ActionGoalHandle = ActionGoalHandle(ros_gh, self._event_loop)
        self._event_loop.call_soon_threadsafe(self.input_data, goal_handle)
        return goal_handle._wait_for_completion()

    @property
    def name(self) -> str:
        try:
            return f"ROS2-ACT-SRV-{self._ros_server.action_type.__name__}-{self._action_name}"
        except Exception:
            return f"ROS2-ACT-SRV-{self._action_name}"

    def close(self) -> None:
        super().close()
        try:
            self._ros_server.destroy()
        except Exception as exc:
            logger.debug("Ignoring ActionServer destroy error: %s", exc)


# ── ActionClient ──────────────────────────────────────────────────────────────


class ActionClient(Generic[_GoalT, _FeedbackT, _ResultT]):
    """Async ROS 2 action client.

    Mirrors the service ``Client`` API: ``send_goal()`` returns an awaitable
    ``ClientGoalHandle``, while ``call()`` is a convenience that sends a goal
    and returns the final result directly.

    Args:
        action_type: The action interface class (e.g. ``Fibonacci``).
        action_name: ROS 2 action name.
        session: ROS session.  Defaults to the current lexical session.
        scope: afor Scope for lifecycle management.

    Example::

        client = ActionClient(Fibonacci, "fibonacci")
        await client.wait_for_server()
        result = await client.call(Fibonacci.Goal(order=10))
        print(result.sequence)
    """

    def __init__(
        self,
        action_type,
        action_name: str,
        session: Optional[BaseSession] = None,
        *,
        scope: Optional[Scope] = AUTO_SCOPE,
    ) -> None:
        self._session: BaseSession = auto_session(session)
        self._event_loop: AbstractEventLoop = asyncio.get_event_loop()
        self._action_type = action_type
        self._action_name = action_name
        self._ros_client: RosActionClient = self._create_ros_client()
        self._scope: Optional[Scope] = None
        self._closed = False
        if scope is AUTO_SCOPE:
            scope = Scope.current(default=None)
        if scope is not None:
            self.attach(scope)

    def _create_ros_client(self) -> RosActionClient:
        with self._session.lock() as node:
            client = RosActionClient(node, self._action_type, self._action_name)
        return client

    def attach(self, scope: Scope) -> None:
        """Attach this client to an active scope for lifecycle management."""
        if self._scope is not None:
            raise RuntimeError(
                f"ActionClient '{self.name}' already attached to a scope"
            )
        self._scope = scope
        assert scope.exit_stack is not None
        scope.exit_stack.callback(self.close)

    async def wait_for_server(self, polling_rate: float = 0.25) -> None:
        """Poll until an action server is available."""
        logger.debug("%s waiting for server", self.name)
        while not self._ros_client.server_is_ready():
            await asyncio.sleep(polling_rate)
        logger.debug("%s server is ready", self.name)

    def send_goal(
        self,
        goal: _GoalT,
    ) -> "ClientGoalHandle[_GoalT, _FeedbackT, _ResultT]":
        """Send a goal and return a ClientGoalHandle immediately.

        The returned handle exposes ``accepted`` and ``result`` futures plus a
        ``feedback`` subscriber.  Returning synchronously lets callers schedule
        multiple goals first and then decide the order in which they await
        acceptance, feedback, or results.

        Args:
            goal: Goal to send.

        Returns:
            ClientGoalHandle.
        """
        logger.debug("%s sending goal", self.name)
        loop = self._event_loop
        handle: ClientGoalHandle = ClientGoalHandle(loop, scope=None)

        def _bridge_feedback(fb_msg) -> None:
            handle.feedback.input_data(fb_msg.feedback)

        ros_fut = self._ros_client.send_goal_async(
            goal, feedback_callback=_bridge_feedback
        )
        ros_gh_fut = asyncify_future(ros_fut, loop)
        handle._watch_goal_response(ros_gh_fut)
        return handle

    async def call(self, goal: _GoalT) -> _ResultT:
        """Send a goal and await the final result, discarding all feedback.

        Convenience wrapper around ``send_goal`` + ``ClientGoalHandle.get_result``.
        Use ``send_goal()`` directly if you need to stream feedback or cancel
        mid-flight.

        Args:
            goal: Goal message to send (e.g. ``Fibonacci.Goal(order=10)``).

        Returns:
            The result message from the action server.

        Raises:
            RuntimeError: If the server rejects the goal.
        """
        gh: ClientGoalHandle = self.send_goal(goal)
        if not await gh.accepted:
            raise RuntimeError(f"{self.name}: goal was rejected by server")
        return await gh.result

    @property
    def name(self) -> str:
        return f"ROS2-ACT-CLI-{self._action_name}"

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._ros_client.destroy()
        except Exception as exc:
            logger.debug("Ignoring ActionClient destroy error: %s", exc)

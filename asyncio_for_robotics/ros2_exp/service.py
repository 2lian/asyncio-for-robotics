from __future__ import annotations

import asyncio
from typing import Generic, Optional, Protocol, TypeVar

from rclpy.experimental import AsyncNode
from rclpy.experimental.async_client import AsyncClient
from rclpy.experimental.async_service import AsyncService
from rclpy.qos import QoSProfile, qos_profile_system_default

from ..core.scope import AUTO_SCOPE, Scope
from ..core.sub import BaseSub
from ..ros2.utils import TopicInfo
from .session import auto_session

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")


class ServiceType(Protocol[_ReqT, _ResT]):
    Request: type[_ReqT]
    Response: type[_ResT]


class Responder(Generic[_ReqT, _ResT]):
    def __init__(
        self,
        request: _ReqT,
        response: _ResT,
        service: AsyncService[_ReqT, _ResT],
        response_ready: asyncio.Future[_ResT],
    ) -> None:
        self.request = request
        self.response = response
        self._service = service
        self._response_ready = response_ready

    def send(self, response: Optional[_ResT] = None) -> None:
        if response is None:
            response = self.response
        if not isinstance(response, self._service.srv_type.Response):
            raise TypeError(
                f"response must be {self._service.srv_type.Response}, "
                f"not {type(response)}"
            )
        if self._response_ready.done():
            raise RuntimeError("This service request is no longer pending")
        self._response_ready.set_result(response)


class Server(BaseSub[Responder[_ReqT, _ResT]], Generic[_ReqT, _ResT]):
    def __init__(
        self,
        msg_type: type[ServiceType[_ReqT, _ResT]],
        topic: str,
        qos_profile: QoSProfile = qos_profile_system_default,
        session: Optional[AsyncNode] = None,
        *,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        self.session: AsyncNode = self._resolve_session(session)
        self.topic_info: TopicInfo = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.srv: AsyncService[_ReqT, _ResT] = self._resolve_sub(self.topic_info)
        super().__init__(scope=scope)

    def _resolve_session(self, session: Optional[AsyncNode]) -> AsyncNode:
        return auto_session(session)

    def _resolve_sub(
        self, topic_info: TopicInfo
    ) -> AsyncService[_ReqT, _ResT]:
        return self.session.create_service(
            srv_type=topic_info.msg_type,
            srv_name=topic_info.topic,
            qos_profile=topic_info.qos,
            callback=self._incoming_request,
            concurrent=True,
        )

    @property
    def name(self) -> str:
        return f"ROS2-EXP-SRV-{self.topic_info.topic}"

    async def _incoming_request(self, request: _ReqT, response: _ResT) -> _ResT:
        response_ready = asyncio.get_running_loop().create_future()
        responder = Responder(request, response, self.srv, response_ready)
        self._input_data_guarded(responder)
        return await response_ready

    def close(self) -> None:
        if not self._closed.is_set():
            self.srv.destroy()
        super().close()


class Client(Generic[_ReqT, _ResT]):
    def __init__(
        self,
        msg_type: type[ServiceType[_ReqT, _ResT]],
        topic: str,
        qos_profile: QoSProfile = qos_profile_system_default,
        session: Optional[AsyncNode] = None,
        *,
        scope: Scope | None = AUTO_SCOPE,
    ) -> None:
        self.session: AsyncNode = self._resolve_session(session)
        self.topic_info: TopicInfo = TopicInfo(
            topic=topic, msg_type=msg_type, qos=qos_profile
        )
        self.cli: AsyncClient[_ReqT, _ResT] = self._resolve_sub(self.topic_info)
        self._scope: Scope | None = None
        self._closed = False
        if scope is AUTO_SCOPE:
            scope = Scope.current(default=None)
        if scope is not None:
            self.attach(scope)

    def _resolve_session(self, session: Optional[AsyncNode]) -> AsyncNode:
        return auto_session(session)

    def _resolve_sub(
        self, topic_info: TopicInfo
    ) -> AsyncClient[_ReqT, _ResT]:
        return self.session.create_client(
            srv_type=topic_info.msg_type,
            srv_name=topic_info.topic,
            qos_profile=topic_info.qos,
        )

    @property
    def name(self) -> str:
        return f"ROS2-EXP-CLI-{self.topic_info.topic}"

    def attach(self, scope: Scope) -> None:
        if self._scope is not None:
            raise RuntimeError(f"Client '{self.name}' already attached to a scope")
        self._scope = scope
        assert scope.exit_stack is not None
        scope.exit_stack.callback(self.close)

    async def wait_for_service(self, polling_rate: float = 0.25) -> None:
        await self.cli.wait_for_service(check_interval=polling_rate)

    def call(self, request: _ReqT) -> asyncio.Future[_ResT]:
        if self._closed:
            raise RuntimeError("Calling a destroyed client is forbidden")

        future = asyncio.get_running_loop().create_future()
        sequence_number = self.cli.handle.send_request(request)
        self.cli._pending_requests[sequence_number] = future
        future.add_done_callback(
            lambda _: self.cli._pending_requests.pop(sequence_number, None)
        )
        return future

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.cli.destroy()

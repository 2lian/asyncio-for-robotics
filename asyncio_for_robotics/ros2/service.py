import asyncio
import logging
from asyncio import AbstractEventLoop, Future
from typing import Generic, Optional, Protocol, TypeVar

from rclpy.client import Client as RosClient
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.service import Service as RosService
from rclpy.task import Future as RosFuture

from ..core.sub import BaseSub
from .future import asyncify_future
from .session import BaseSession, auto_session
from .utils import QOS_DEFAULT, TopicInfo

logger = logging.getLogger(__name__)

_ReqT = TypeVar("_ReqT")
_ResT = TypeVar("_ResT")


# Generic base class for a service definition
class ServiceType(Protocol[_ReqT, _ResT]):
    Request: type[_ReqT]
    Response: type[_ResT]


_SrvType = ServiceType


class Responder(Generic[_ReqT, _ResT]):
    def __init__(
        self,
        request: _ReqT,
        response: _ResT,
        srv: RosService,
        event_loop: AbstractEventLoop,
    ) -> None:
        """Sends a response to a request.

        Use:
          - Get request data: `the_request = responder.request`
          - Set the response: `responder.response = my_response`
          - Send the response: `responder.send()`

        .. Important:
            User should not instanciate this object.

        .. Important:
            It is possible to send several replies. Do not.
        """
        self.request: _ReqT = request
        self.response: _ResT = response
        self._srv: RosService = srv
        self._event_loop = event_loop
        self._header_ready: Future = Future(loop=event_loop)

    def _set_header(self, header) -> None:
        logger.debug(f"Header set in respondable obj")
        self._header_ready.set_result(header)

    def send(self, response: Optional[_ResT] = None) -> None:
        """Sends the response on the transport (rmw)

        This reimplement the `Service.send_response` of ros, because we overode
        it previously with `response_overide`.

        This needs to be async because we need to wait for the header data.
        """
        if response is None:
            response = self.response
        logger.debug("Properly replying to request")
        if not self._header_ready.done():
            raise AttributeError(f"Header is missing! something went terribly wrong")
        header = self._header_ready.result()
        if not isinstance(
            response,
            self._srv.srv_type.Response,  # type: ignore
        ):
            raise TypeError(
                f"Response is of the wrong type: \n  - {type(response)=}\n  - {self._srv.srv_type.Response=}"
            )
        with self._srv.handle:
            c_implementation = self._srv._Service__service  # type: ignore
            if isinstance(header, _rclpy.rmw_service_info_t):
                c_implementation.service_send_response(response, header.request_id)
            elif isinstance(header, _rclpy.rmw_request_id_t):
                c_implementation.service_send_response(response, header)
            else:
                raise TypeError(f"Header is of the wrong type: {type(header)=}")


def response_overide(response: Responder, header) -> None:
    """Replaces the callback behavior of ros.

    By default when service callback is triggered, the response is returned,
    then passed to `Service.send_response` that then sends it on the tranport
    (rmw). However, we don't want to respond yet, we wanna respond later. So we
    overide Service.send_response with nothing (this function)...

    Not exactly nothing. `Service.send_response` has the critical "header"
    information that we need to save. This header is saved in the Responder
    instance later used when the user sends the response.
    """
    responder = response
    logger.debug("Response and header intercepted")
    responder._event_loop.call_soon_threadsafe(responder._set_header, header)


class Server(BaseSub[Responder[_ReqT, _ResT]]):
    def __init__(
        self,
        msg_type: type[_SrvType[_ReqT, _ResT]],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Optional[BaseSession] = None,
    ) -> None:
        """Implements an async ROS 2 service server.

        This is a standard `afor` subscriber, where the data stream is made of
        `Responder` objects. The `Responder` holds the request and response.
        The response is then send using `Responder.send()`

        Args:
            msg_type:
            topic:
            qos_profile:
            session:
        """
        self.session: BaseSession = self._resolve_session(session)
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.srv = self._resolve_sub(self.topic_info)
        super().__init__()

    @property
    def name(self) -> str:
        try:
            return f"ROS2-SRV-{self.srv.srv_name}"
        except:
            return f"ROS2-SRV-{self.topic_info.topic}"

    def _resolve_session(self, session: Optional[BaseSession]) -> BaseSession:
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo) -> RosService:
        logger.debug("%s requesting lock for creation", self.name)
        with self.session.lock() as node:
            serv_modified = node.create_service(
                srv_type=topic_info.msg_type,
                srv_name=topic_info.topic,
                qos_profile=topic_info.qos,
                callback=self._incomming_request_cbk,
            )
            ### VVV IMPORTANT VVV ###
            ###                   ###
            serv_modified.send_response = response_overide
            ###                   ###
            ### ^^^           ^^^ ###
        return serv_modified

    def _incomming_request_cbk(self, request: _ReqT, response: _ResT) -> Responder:
        """Fake service callback

        actually returning the responder to be later intercepted, and the
        header to be later set.
        """
        responder_for_user: Responder[_ReqT, _ResT] = Responder(
            request, response, self.srv, self._event_loop
        )

        def execute_in_asyncio_thread():
            responder_for_user._header_ready.add_done_callback(
                lambda *_: self._differed_header_ready_cbk(responder_for_user)
            )

        self._event_loop.call_soon_threadsafe(execute_in_asyncio_thread)
        return responder_for_user

    def _differed_header_ready_cbk(self, responder_for_user: Responder):
        """Executes once the header is set.

        Once the header of the request is set, the responder is fully usable.
        This needs to be differed in a cbk because it happens after the return
        statement of _incomming_request_cbk
        """
        try:
            healty = self.input_data(responder_for_user)
            if not healty:
                self.session._node.destroy_service(self.srv)
        except Exception as e:
            logger.error(e)

    def close(self):
        with self.session.lock() as node:
            if not node.executor.context.ok():
                return
            node.destroy_service(self.srv)


class Client(Generic[_ReqT, _ResT]):
    def __init__(
        self,
        msg_type: type[_SrvType[_ReqT, _ResT]],
        topic: str,
        qos_profile: QoSProfile = QOS_DEFAULT,
        session: Optional[BaseSession] = None,
    ) -> None:
        """Implements an async ROS 2 service client.

        The `Client.call(...)` method will send a request and return its
        asyncio.Future response.
        """
        self.session: BaseSession = self._resolve_session(session)
        self._event_loop = asyncio.get_event_loop()
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.cli: RosClient = self._resolve_sub(self.topic_info)

    def _resolve_session(self, session: Optional[BaseSession]) -> BaseSession:
        return auto_session(session)

    def _resolve_sub(self, topic_info: TopicInfo) -> RosClient:
        logger.debug("%s requesting lock for creation", self.name)
        with self.session.lock() as node:
            client = node.create_client(
                srv_type=topic_info.msg_type,
                srv_name=topic_info.topic,
                qos_profile=topic_info.qos,
            )
        return client

    async def wait_for_service(self, polling_rate: float = 0.25):
        """
        Wait for a service server to become ready.

        .. Note:
            By default in ROS 2 this is a busy wait in a while loop. Crazy unga
            bunga. Me too unga bunga.

        Args:
            polling_rate: Rate (in s) at which to check for readiness.

        Returns:
            As soon as a server becomes ready.
        """
        logger.debug("%s waiting for server", self.name)
        while 1:
            if self.cli.service_is_ready():
                logger.debug("%s server is here", self.name)
                return
            else:
                await asyncio.sleep(polling_rate)

    def call(self, req: _ReqT) -> Future[_ResT]:
        """Calls the service and returns the response as asyncio.Future

        Args:
            req: Request to send to the server.

        Returns:
            response as an asyncio.Future
        """
        logger.debug("%s making request", self.name)

        def dbg(ros_response):
            logger.debug("%s got response", self.name)

        # lock not necessary, ros seems safe
        ros_fut: RosFuture = self.cli.call_async(req)
        future: Future[_ResT] = asyncify_future(ros_fut, self._event_loop)
        future.add_done_callback(dbg)
        return future

    @property
    def name(self) -> str:
        try:
            return f"ROS2-CLI-{self.cli.srv_name}"
        except:
            return f"ROS2-CLI-{self.topic_info.topic}"

    def close(self):
        with self.session.lock() as node:
            if not node.executor.context.ok():
                return
            node.destroy_client(self.cli)

import asyncio
import logging
from asyncio import AbstractEventLoop, Future
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    Optional,
    Protocol,
    Self,
    Type,
    TypeVar,
    Union,
)

from rclpy.client import Client as RosClient
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.service import Service as RosService
from rclpy.subscription import Subscription
from rclpy.task import Future as RosFuture

from ..core.sub import BaseSub
from .session import BaseSession, auto_session
from .utils import QOS_DEFAULT, TopicInfo, _MsgType

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
        self._sub: RosService = srv
        self._event_loop = event_loop
        self._header_ready: Future = Future(loop=event_loop)

    def _set_header(self, header) -> None:
        logger.debug(f"Header set in respondable obj")
        self._header_ready.set_result(header)

    async def send(self) -> None:
        """Sends the response on the transport (rmw)

        This reimplement the `Service.send_response` of ros, because we overode
        it previously with `response_overide`.

        This needs to be async because we need to wait for the header data.
        """
        logger.debug("Properly replying to request")
        header = await self._header_ready
        if not isinstance(
            self.response,
            self._sub.srv_type.Response,  # type: ignore
        ):
            raise TypeError()
        with self._sub.handle:
            c_implementation = self._sub._Service__service  # type: ignore
            if isinstance(header, _rclpy.rmw_service_info_t):
                c_implementation.service_send_response(self.response, header.request_id)
            elif isinstance(header, _rclpy.rmw_request_id_t):
                c_implementation.service_send_response(self.response, header)
            else:
                raise TypeError()


def response_overide(response: Responder, header) -> None:
    """Replaces the callback behavior of ros.

    By default when service callback is triggered, the response is returned,
    then passed to `Service.send_response` that then sends it on the tranport
    (rmw). However, we don't want to respond yet, we wanna respond later. So we
    overide Service.send_response with nothing...

    Not exactly nothing. `Service.send_response` has the critical "header"
    information that we need to save. This is used later in our proper
    response. This header is saved in the Responder instance later used by the
    user to send the response.
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
        buff_size: int = 10,
    ) -> None:
        self.session: BaseSession = self._resolve_session(session)
        self.topic_info = TopicInfo(topic=topic, msg_type=msg_type, qos=qos_profile)
        self.srv = self._resolve_sub(self.topic_info)
        super().__init__(buff_size)

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
                callback=self.callback_for_sub,
            )
            ### VVV IMPORTANT VVV ###
            ###                   ###
            serv_modified.send_response = response_overide
            ###                   ###
            ### ^^^           ^^^ ###
        return serv_modified

    def callback_for_sub(self, request: _ReqT, response: _ResT) -> Responder:
        # logger.debug("%s got request", self.name)
        responder_for_user: Responder[_ReqT, _ResT] = Responder(
            request, response, self.srv, self._event_loop
        )
        try:
            healty = self.input_data(responder_for_user)
            if not healty:
                self.session._node.destroy_service(self.srv)
        except Exception as e:
            logger.error(e)
        return responder_for_user

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
        buff_size: int = 10,
    ) -> None:
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

    def call(self, req: _ReqT) -> Future[_ResT]:
        response: Future[_ResT] = Future()

        def ros_cbk(ros_response):
            logger.debug("%s got response", self.name)
            self._event_loop.call_soon_threadsafe(
                response.set_result, ros_response.result()
            )

        # lock not necessary, ros seems safe
        logger.debug("%s making request", self.name)
        fut: RosFuture = self.cli.call_async(req)
        fut.add_done_callback(ros_cbk)
        return response

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

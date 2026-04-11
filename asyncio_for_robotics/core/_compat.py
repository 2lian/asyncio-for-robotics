try:
    from asyncio import TaskGroup
except ImportError:
    from taskgroup import TaskGroup

try:
    BaseExceptionGroup = BaseExceptionGroup
except NameError:
    from exceptiongroup import BaseExceptionGroup

try:
    from asyncio import TaskGroup
except ImportError:
    try:
        from taskgroup import TaskGroup
    except ImportError:
        TaskGroup = NotImplemented

try:
    BaseExceptionGroup = BaseExceptionGroup
except NameError:
    # BaseExceptionGroup = NotImplemented
    from exceptiongroup import BaseExceptionGroup

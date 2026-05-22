"""
引用误报停用词表。
可按目标系统扩展：继承 BaseStoplist 并添加领域特定词汇。
"""

from __future__ import annotations


class BaseStoplist:
    """基础停用词表 — 通用的误报过滤词"""

    # 通用类型名 / 编程关键词
    GENERIC_TYPES = {
        "FLOAT", "DOUBLE", "INT32", "UINT8", "UINT16", "UINT32", "UINT64",
        "INT8", "INT16", "BOOL", "STRING", "CHAR", "VOID",
        "Float", "Double", "Int32", "String", "Bool",
        "Float32", "Float64",
    }

    # 常见英文大写词（被 UPPER_CASE 正则误匹配）
    COMMON_WORDS = {
        "ONLY", "READ", "WRITE", "THIS", "THAT", "THEN", "WHEN", "WITH",
        "FROM", "INTO", "OVER", "UNDER", "AFTER", "BEFORE", "BETWEEN",
        "TRUE", "FALSE", "NULL", "NONE", "TODO", "NOTE", "DEPRECATED",
        "IMPORTANT", "WARNING", "REQUIRED", "OPTIONAL", "DEFAULT",
        "ENABLED", "DISABLED", "UNKNOWN", "INVALID", "VALID",
        "MAX", "MIN", "ALL", "ANY", "SET", "GET", "NEW", "OLD",
    }

    # 过短或过通用的 CamelCase
    GENERIC_CAMEL = {
        "NaN", "Inf", "The", "For", "Not", "And", "But",
        "Use", "See", "Can", "May", "Has", "Get", "Set",
    }

    @classmethod
    def all_stopwords(cls) -> set[str]:
        return cls.GENERIC_TYPES | cls.COMMON_WORDS | cls.GENERIC_CAMEL

    @classmethod
    def is_stopword(cls, word: str) -> bool:
        return word in cls.all_stopwords()


class PX4Stoplist(BaseStoplist):
    """PX4 特定的停用词（非实体但被正则匹配到的词）"""

    PX4_SPECIFIC = {
        "GNSS", "SITL", "RTPS", "UAVCAN", "VTOL", "HITL",
        "NED", "ENU", "WGS84", "ECEF",
        "ASCII", "JSON", "XML", "YAML",
        "GPIO", "SPI", "UART", "USB", "CAN",
        "PID", "EKF", "LPE", "INAV",
    }

    @classmethod
    def all_stopwords(cls) -> set[str]:
        return super().all_stopwords() | cls.PX4_SPECIFIC


def get_stoplist(target: str = "default") -> set[str]:
    """获取目标系统对应的停用词集合"""
    stoplists = {
        "default": BaseStoplist,
        "px4": PX4Stoplist,
    }
    cls = stoplists.get(target, BaseStoplist)
    return cls.all_stopwords()

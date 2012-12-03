#define BEHOLDER_VERSION 

#define BH_DEBUG_LVL 4
#if BH_DEBUG_LVL > 0
    #define BH_INFO
#endif
#if BH_DEBUG_LVL > 1
    #define BH_LOG
#endif
#if BH_DEBUG_LVL > 2
    #define BH_VERBOSE
#endif
# if BH_DEBUG_LVL > 3
    #define BH_FLOOD
#endif


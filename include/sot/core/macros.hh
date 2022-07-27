#ifndef __SOT_CORE_MACROS_HH__
#define __SOT_CORE_MACROS_HH__

// ref https://www.fluentcpp.com/2019/08/30/how-to-disable-a-warning-in-cpp/
#if defined(_MSC_VER)

#define SOT_CORE_DISABLE_WARNING_PUSH __pragma(warning(push))
#define SOT_CORE_DISABLE_WARNING_POP __pragma(warning(pop))
#define SOT_CORE_DISABLE_WARNING(warningNumber) \
  __pragma(warning(disable : warningNumber))
#define SOT_CORE_DISABLE_WARNING_DEPRECATED SOT_CORE_DISABLE_WARNING(4996)
#define SOT_CORE_DISABLE_WARNING_FALLTHROUGH

#elif defined(__GNUC__) || defined(__clang__)

#define SOT_CORE_DO_PRAGMA(X) _Pragma(#X)
#define SOT_CORE_DISABLE_WARNING_PUSH SOT_CORE_DO_PRAGMA(GCC diagnostic push)
#define SOT_CORE_DISABLE_WARNING_POP SOT_CORE_DO_PRAGMA(GCC diagnostic pop)
#define SOT_CORE_DISABLE_WARNING(warningName) \
  SOT_CORE_DO_PRAGMA(GCC diagnostic ignored #warningName)
#define SOT_CORE_DISABLE_WARNING_DEPRECATED \
  SOT_CORE_DISABLE_WARNING(-Wdeprecated - declarations)
#define SOT_CORE_DISABLE_WARNING_FALLTHROUGH \
  SOT_CORE_DISABLE_WARNING(-Wimplicit - fallthrough)

#else

#define SOT_CORE_DISABLE_WARNING_PUSH
#define SOT_CORE_DISABLE_WARNING_POP
#define SOT_CORE_DISABLE_WARNING_DEPRECATED

#endif

#endif

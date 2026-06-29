Import("env")

# PlatformIO 默认会给 ld 加 -Wl,-Map=.../firmware.map
# 在部分 Windows + 中文/括号路径下会导致 “cannot open map file ... No such file or directory”
# 这里直接把该参数移除，保证项目无需改路径也能正常链接。

def _filter_linkflags(flags):
    out = []
    for f in flags:
        s = str(f)
        if s.startswith("-Wl,-Map=") or s.endswith("firmware.map"):
            continue
        out.append(f)
    return out

env.Replace(LINKFLAGS=_filter_linkflags(env.get("LINKFLAGS", [])))


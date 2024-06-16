Import("env")
Import("projenv")
for e in (env, projenv):
    e.Append(CXXFLAGS=["-Wno-register"])

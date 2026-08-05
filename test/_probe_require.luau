local ok, mod = pcall(require, "../src/runtime")
print("runtime:", ok, type(mod))

## 一、
**1.张量**
简单赋值为浅拷贝：
```
vals = np.array([1, 2, 3])
src = o3c.Tensor(vals)
dst = src
src[0] += 10

# Changes in one will get reflected in other.
print("Source tensor:\n{}".format(src))
print("\nTarget tensor:\n{}".format(dst))```

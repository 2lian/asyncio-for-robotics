The --bump option supports the following common version components: major, minor, patch, stable, alpha, beta, rc, post, and dev. When provided more than once, the components will be applied in order, from largest (major) to smallest (dev).

```bash
uv version --bump patch
```

```bash
uv build --no-sources
```

```bash
uv publish --index testpypi
```


Install from testpypi with

```bash
uv pip install --index-url https://test.pypi.org/simple/   --extra-index-url https://pypi.org/simple   asyncio-for-robotics[dev,build,zenoh]
```

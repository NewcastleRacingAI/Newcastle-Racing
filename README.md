# Newcastle Racing

## Requirements

- [uv](https://docs.astral.sh/uv/)
   - Can be installed with pip globally

## Use

Recommended, although not mandatory: create a virutal environment.

```bash
python3 -m venv .venv
# Linux 
source .venv/bin/activate
# Windows
.venv/Script/activate
```

We use [uv](https://docs.astral.sh/uv/) to manage the dependencies.
Hence, you need uv installed, either globally or in the virtual environment:

```bash
pip install uv
```

Lastly, we can run the full script:

```bash
uv run main.py -h
```

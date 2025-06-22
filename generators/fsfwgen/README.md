# FSFW Generators

Generic Python module to generate source code or information for the
Flight Software Framework.

Currently, this includes the following helper modules:

1. `events` to generate Event translation source files and CSV lists
2. `returnvalues` to generate Returnvalue translation source files and CSV lists
3. `objects` to generate Object ID translation files and CSV lists

## Installing

It is recommended to use a virtual environment

**Linux**

```sh
python3 -m venv venv
. venv/bin/activate
```

**Windows**

```sh
py -m venv venv
. venv/bin/activate
```

Then you can install the package with

```sh
python -m pip install .
```

You can add `-e` after `install` to perform an interactive/debug installation.
This is recommended if you debugging, planning to extend the script or
performing changes.

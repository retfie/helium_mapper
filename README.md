## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder where the
`helium_mapper` and needed Zephyr modules will be cloned. You can do
that by running:

```shell
# initialize workspace for the helium_mapper (main branch)
west init -m https://github.com/retfie/helium_mapper --mr main helium_mapper_project
# update Zephyr modules
cd helium_mapper_project/helium_mapper
west update
```

### Build & Run

The application can be built by running:

```shell
west build -b rak4631_nrf52840 -s app
```

Once you have built the application you can flash it by running:

```shell
west flash
```

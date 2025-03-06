# ESP-BOX Fluidsim

Example repository using espp components to have an interactive fluid simulation
on the ESP32-S3-BOX and ESP32-S3-BOX-3, using the IMU to measure the gravity
vector and using the touchscreen for other fluid interactions.

https://github.com/user-attachments/assets/d6483866-30f6-4c09-b3e8-c47c18a84001

![image](https://github.com/user-attachments/assets/19921856-58a1-4680-b37f-32216320a8ce)

https://github.com/user-attachments/assets/37ce65c0-ff92-4d34-8756-d0fdd518413c

This repository implements an embedded c++ (header-only) version of [Ten Minute
Physics' FLIP Fluid
Simulation](https://matthias-research.github.io/pages/tenMinutePhysics/index.html).

This project was inspired by
https://hackaday.io/project/202470-esp32-fluid-simulation-on-16x16-led-matrix.

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules https://github.com/finger563/esp-box-fluidsim
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

![image](https://github.com/user-attachments/assets/26bc0268-bc53-4949-91d0-7f540449c2ac)
![image](https://github.com/user-attachments/assets/19921856-58a1-4680-b37f-32216320a8ce)
![image](https://github.com/user-attachments/assets/3f373d48-71a2-4acc-9d72-eb133fbd9ee0)

https://github.com/user-attachments/assets/4c19dea8-bcb9-4b38-93b3-ad01f3c6f904

https://github.com/user-attachments/assets/d6483866-30f6-4c09-b3e8-c47c18a84001

https://github.com/user-attachments/assets/37ce65c0-ff92-4d34-8756-d0fdd518413c


## Developing

If you're developing code for this repository, it's recommended to configure
your development environment:

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied, by running
`clang-format` each time you change the code (via a git pre-commit hook) using
the [./.clang-format](./.clang-format) code style configuration file.

If you ever want to re-run the code formatting on all files in the repository,
you can do so:

``` console
pre-commit run --all-files
```

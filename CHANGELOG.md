# Changelog

## [0.5.0](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/compare/project-emerge-firmware-v0.4.1...project-emerge-firmware-v0.5.0) (2026-09-05)


### Features

* implement motors configuration via mqtt ([8d442ec](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/8d442ecbddc84c9672fab5cc354819abe0e31529))


### Bug Fixes

* prevent error connection to restored broker mqtt ([cc4b325](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/cc4b32593ba0a8895bed6549eb8f96c2b67d5cbb))

## [0.4.1](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/compare/project-emerge-firmware-v0.4.0...project-emerge-firmware-v0.4.1) (2026-09-04)


### Bug Fixes

* avoid motor starving due to friction on low PWM values ([d8b1cdc](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/d8b1cdc7c3b9c2c0eec616a02673099139adc056))

## [0.4.0](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/compare/project-emerge-firmware-v0.3.0...project-emerge-firmware-v0.4.0) (2026-08-18)


### Features

* automatize ota updates ([d1b2510](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/d1b251088f337d939ed87cba15ae2377980d8bae))
* implement uwb tracking ([1e3d0da](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/1e3d0dad7623e71ded575b7dc219ffc81d89cad2))
* remove UWB ([347c177](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/347c177a8d3ffe415319f203e70ecd348172be85))


### Bug Fixes

* resolve clippy warnings from laze build -b dropbot clippy ([8021a0d](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/8021a0d2de087ae2fd200fcf34287395bc8c3b2b))

## [0.3.0](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/compare/project-emerge-firmware-v0.2.0...project-emerge-firmware-v0.3.0) (2026-08-10)


### Features

* add display driver and minimal setup ([d1000ee](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/d1000eedf3c32703122386576b0bfc79b676316a))
* add mqtt telemetry and refactoring tasks ([8520bee](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/8520beed80b0538b81f7ab9b1248d9f972893374))
* add robot number in the display ([88d27d1](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/88d27d1d54c9e40caf5d5ef91bbcc9d9f2329346))
* connection to mqtt broker ([cf6340a](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/cf6340a28f4111621dbc01ce755dce48d1f85a2a))
* implement 9-axis IMU ([9d515b7](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/9d515b7975f9f9c21ad85f9b902b48221dc9667e))
* implement motors command ([1413ce8](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/1413ce84d5cff88aad727662c8eac034c7d608e7))
* implement OTA updates ([6217edc](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/6217edcc6d3bec048eb592f03e2650f072bac572))
* implement power button and menus ([ad2f137](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/ad2f1377f4a0e899f1b70755b039a344cc281517))
* set env variable for set mqtt broker ([ca7a7df](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/ca7a7dfbc576d272f962204df563368ab4577dc6))
* set firmware version form Cargo.toml ([075c8fe](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/075c8fec02d4bfab751cfb4911d541907ceb551f))
* setup mqtt ([4ec1a53](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/4ec1a5304a1e73c231e54eb59d8dbf3e912fdd66))


### Bug Fixes

* resolve silent merge corruption breaking the dropbot build ([6c77693](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/6c77693126d7e76a0e5a71d3ab2ba2a25e617958))
* some fixed in motor driver ([0a54c82](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/0a54c825d816d1d08a4c5fe78f038c3a0d07e3df))
* start time for PWM module ([751da6c](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/751da6c76b66c201ebe4fb6731dc9b5738910c9d))

## [0.2.0](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/compare/project-emerge-firmware-v0.1.0...project-emerge-firmware-v0.2.0) (2026-06-21)


### Features

* add ota update ([2c6ca49](https://github.com/Project-Emerge/Project-Emerge-dropbot-firmware/commit/2c6ca496c5d27a4344eb32c164126caed81a094c))

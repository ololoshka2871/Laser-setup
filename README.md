# README
Плата управления камерой точной настройки кварцевых резонаторов.

# Build and run
Use [probe-run](https://github.com/knurling-rs/probe-run) and [flip-link](https://github.com/knurling-rs/flip-link)
1. `cargo {rb,rrb}` - flash debug/release and run
2. `cargo br` - view the [bloat](https://github.com/RazrFalcon/cargo-bloat) report

# Connection

### USB
| PIN | Назначение |
| --- | --- |
| PB8 | USB_EN |
| PA11 | USB_DM |
| PB12 | USB_DP |

### Реле актуатора
PB11

### Клапан вакуум/атмосфера
PB10

### Управление мультиплексором резонаторов
| PIN | Назначение |
| --- | --- |
| PA5 | CLK |
| PA6 | LAT |
| PA7 | DATA |

### Энкодер
| PIN | Назначение |
| --- | --- |
| PA1 | ENC1 |
| PA2 | ENC2 |
| PA3 | ENC_BT |

### Дисплей
| PIN | Назначение |
| --- | --- |
| PB6 | SCL |
| PB7 | SDA |

## Power Draw Tests

Power draw tests done on 01/25/2026 by @nihonjinrxs.

**Baseline power draw: 0.582 A**

> Baseline power draw includes control system power, power switch light, auto mode light, and lights on color sensors (not indicators, but the sensors themselves)

| __*Subsystem*__   | __*Setting*__                     | __*Robot Power Draw* (A)__ | __*Delta* (A)__ | __*Notes*__                                     |
|-------------------|-----------------------------------|---------------------------:|----------------:|-------------------------------------------------|
| `IndicatorLights` | +1 light on                       |                      0.776 |         + 0.194 | Tested purple, green, red; all same             |
| `Lift`            | Engaging/lifting                  |                      2.030 |         + 1.448 |                                                 |
| `Hood`            | Moving                            |                      0.824 |         + 0.242 |                                                 |
| `FeedStop`        | Moving                            |                      0.776 |         + 0.194 |                                                 |
| `Intake`          | Intaking                          |                      2.710 |         + 2.128 |                                                 |
| `Shooter`         | Flywheels @2800 RPM               |                      5.000 |         + 4.418 | Idle RPM, Peak 7.5 A (+6.918 A)                 |
| `Shooter`         | Flywheels @1850 RPM               |                      3.400 |         + 2.818 | At Goal Preset RPM                              |
| `Shooter`         | Flywheels @2225 RPM               |                      3.600 |         + 3.018 | Mid Near Zone Preset RPM                        |
| `Shooter`         | Flywheels @2400 RPM               |                      4.100 |         + 3.518 | End Near Zone Preset RPM                        |
| `Shooter`         | Flywheels @3000 RPM               |                      5.580 |         + 4.998 | Far Wall Preset RPM                             |
| `Drivetrain`      | Rotation @ Full speed             |                      4.630 |         + 4.048 |                                                 |
| `Drivetrain`      | Strafe @ A little over half speed |                      7.782 |         + 7.200 | Limited space, so could not get full speed      |
| `Drivetrain`      | Forward @ Full speed              |                     11.820 |        + 11.238 | Limited space, so very brief, noisy measurement |

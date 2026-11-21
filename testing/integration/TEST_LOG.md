# Integration tests

| Interface | Production tests | Physical evidence |
| --- | --- | --- |
| MQTT, supervisor and motor stop | `test_release_bridge`, `test_network_stop`, `test_mqtt_delivery` | T2-01, T2-08 and T2-09 |
| Barcode, junction and shared motion | `test_rtos_barcode`, `test_junction_sensor`, `test_junction_policy` | Run 4 decoded and completed Z right, Q U-turn and A left |
| IMU, hump and motor assistance | `test_rtos_ramp_bridge`, `test_release_imu`, `test_imu_service_rtos` | Run 4 accepted peaks of 12 mm and 16 mm |
| Obstacle lease, shared motion and line handback | `test_b5_straight_gate`, `test_b5_mission`, `test_nav_steer` | Run 4 completed an obstacle bypass and returned control to Buddy 3 |
| Dashboard, wire packers and stored logs | `test_release_exports`, `test_compact_signed`, `test_wireless_calibration`, `dashboard/test/test_b5_rtos.js` | Run 4 ZIP contains four completed stored datasets and retained live history |

[Physical run 4](../evidence/tribunal2/run-4/README.md) ·
[v0.4.3 automated results](../evidence/tribunal2/v0.4.3/README.md) ·
[Run the suites](../TRIBUNAL_2.md#build-and-repeat-the-automated-checks)

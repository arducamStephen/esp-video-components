# esp-video-components

This is the repository for a some video related components, which aims to be uploaded to [IDF Component Registry](https://components.espressif.com/).


## IMX500 Demo

- Path: esp-video/examples/simple_video_server
- Camera module: Arducam IMX500 Module
- Model: YOLOv8n
- Dataset: COCO80
- Platform: ESP32-P4

| esp32p4 GPIO | imx500 mcu module spi |
|:-|:-|
| 26 |sck |
| 48 |rx |
| 53 |tx |
| 47 |cs |

| Items | fps |
|:-|:-|
| frame（MIPI） | 30 |
| metadata (SPI) | 10 |

## Component list in this project

- esp_cam_sensor
- esp_sccb_intf
- esp_video
- esp_ipa

## License

| Component | License |
|:-|:-|
| esp_cam_sensor | [Apache V2.0 License](esp_cam_sensor/LICENSE) |
| esp_sccb_intf | [Apache V2.0 License](esp_sccb_intf/LICENSE) |
| esp_video | [ESPRESSIF MIT License](esp_video/LICENSE) |
| esp_ipa | [ESPRESSIF MIT License](esp_ipa/LICENSE) |

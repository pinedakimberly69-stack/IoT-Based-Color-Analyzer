# IoT-Based Personal Color Analyzer

An IoT-based personal color analysis system that automates skin undertone detection and seasonal color palette recommendations using an ESP32 microcontroller, TCS34725 RGB color sensor, and Flutter mobile application.

## Project Overview

The IoT-Based Personal Color Analyzer is designed to identify a user's skin undertone and provide personalized color recommendations through a sensor-based analysis system.

The device captures skin color measurements using an RGB color sensor, processes the collected data through an ESP32, and displays the analysis results through a Flutter mobile application.

## Key Features

- **Real-Time Skin Tone Detection** – Captures skin color measurements through the TCS34725 sensor.
- **Undertone Classification** – Identifies Warm, Cool, or Neutral undertones.
- **Seasonal Color Analysis** – Provides seasonal color palette recommendations.
- **Jewelry Recommendations** – Suggests Gold or Silver jewelry based on the analysis.
- **Mobile Application Integration** – Displays analysis results through a Flutter application.
- **Scan Quality Assessment** – Provides confidence, stability, and scan quality indicators.

## Technologies Used

### Hardware
- ESP32 Development Board
- TCS34725 RGB Color Sensor
- Supporting electronic components

### Software
- C++ / Arduino Framework
- PlatformIO
- Flutter
- Dart
- Visual Studio Code
- Android Studio

## System Architecture

The system follows a three-layer architecture:

1. **Input Layer:** The TCS34725 RGB color sensor captures skin color measurements.
2. **Processing Layer:** The ESP32 processes RGB values and calculates skin color characteristics and undertone classification.
3. **Output Layer:** The Flutter mobile application retrieves and displays the analysis results.

## Project Structure

```text
IoT-Based-Color-Analyzer/
│
├── firmware/
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
│
├── mobile_app/
│   ├── lib/
│   │   └── main.dart
│   └── pubspec.yaml
│
└── README.md
```

## Installation and Setup

### ESP32 Firmware

1. Install Visual Studio Code.
2. Install the PlatformIO IDE extension.
3. Open the firmware folder.
4. Install the required libraries specified in platformio.ini.
5. Connect the ESP32 development board.
6. Build and upload the firmware.

### Flutter Mobile Application

1. Install Flutter SDK and Android Studio.
2. Open the mobile_app project.
3. Install project dependencies:

```bash
flutter pub get
```

4. Connect the mobile application to the ESP32 through the configured Wi-Fi connection.
5. Run the application:

```bash
flutter run
```

## Development

This project demonstrates the integration of embedded systems, IoT communication, sensor-based data processing, and mobile application development.

## Project Information

**Project Title:** IoT-Based Personal Color Analyzer for Automated Skin Undertone Analysis and Palette Recommendation

**Field:** Computer Engineering

**Developer:** Kimberly Pineda

**Repository:** IoT-Based-Color-Analyzer

## License

This project is intended for educational, research, and portfolio demonstration purposes.

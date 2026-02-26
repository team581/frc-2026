# Team 581's 2026 Robot Code Monorepo

[![CI](https://github.com/team581/frc-2026/actions/workflows/ci.yml/badge.svg)](https://github.com/team581/frc-2026/actions/workflows/ci.yml)

[Team 581](https://github.com/team581)'s 2026 robot code monorepo.

## Project Structure

This repository is organized as a Gradle monorepo with the following projects:

- **`shared/`** - Shared utility library
- **`turret-bot/`** - Turret testbed for software development
- **`dumper-bot/`** - Triple barrel shooter alpha bot
- **`comp-bot/`** - Competition bot code

## Building and running

### Build

```sh
# Build all projects
./gradlew build

# Build specific project
./gradlew comp-bot:build
```

### Deploy to roboRIO

```sh
./gradlew comp-bot:deploy
```

### Running the simulator

```sh
# Run simulator for specific project

```

### Running tests

```sh
# Run all tests
./gradlew test

# Run tests for specific project
./gradlew comp-bot:test
```

### Code formatting

```sh
# Check formatting
./gradlew spotlessCheck

# Apply formatting
./gradlew spotlessApply
```

## Installing Java

WPILib ships with JDK 17, but we use JDK 21.
These instructions describe how to install JDK 21 and set it up in WPILib VS Code.

### Windows

1. Run `winget install -e --id EclipseAdoptium.Temurin.21.JDK`
2. In VS Code, run "Preferences: Open User Settings (JSON)" from the command palette
3. Delete config values referencing the WPILib JDK
   - `java.jdt.ls.java.home`
   - `java.configuration.runtimes`
   - `terminal.integrated.env.windows` -> `JAVA_HOME`

### macOS

1. Install Brew `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
2. Run `brew bundle`
   - This will install all tools for development, including WPILib itself
   - If you just want to install the JDK, run `brew install openjdk@21`
3. In VS Code, run "Preferences: Open User Settings (JSON)" from the command palette
4. Delete config values referencing the WPILib JDK
   - `java.jdt.ls.java.home`
   - `java.configuration.runtimes`
   - `terminal.integrated.env.osx` -> `JAVA_HOME`

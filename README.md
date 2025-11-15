# Team 581's 2026 Robot Code Monorepo

[![CI](https://github.com/team581/frc-2026/actions/workflows/ci.yml/badge.svg)](https://github.com/team581/frc-2026/actions/workflows/ci.yml)

[Team 581](https://github.com/team581)'s 2026 robot code monorepo.

## Project Structure

This repository is organized as a Gradle monorepo with the following projects:

- **`shared/`** - Shared utility library
- **`turret-bot/`** - Turret bot code

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
./gradlew comp-bot:simulateJava
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

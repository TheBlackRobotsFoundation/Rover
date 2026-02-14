# Changelog

Todos los cambios notables del proyecto Rover se documentan en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Estructura Inicial
- Proyecto modular con componentes independientes
- Scripts de instalación y compilación automatizados
- Estructura de documentación completa
- CI/CD con GitHub Actions
- Configuración de VSCode

### Componentes

#### rover_core
- Paquetes: rover_msgs, rover_interfaces
- Mensajes y servicios personalizados

#### rover_drivers
- Paquetes: roboclaw_driver, ibus_driver, px4_bridge
- Drivers para hardware del rover

#### rover_control
- Paquetes: rover_base_controller, rover_teleop
- Control de base y teleoperación

#### rover_navigation
- Paquetes: rover_nav2_bringup, rover_localization
- Integración con Nav2 y localización

#### rover_perception
- Estructura lista para sensores LIDAR y cámara

#### rover_simulation
- Paquetes: rover_description, rover_gazebo
- URDF y simulación Gazebo

#### rover_bringup
- Paquetes: rover_launch, rover_config
- Launch files y configuración centralizada

#### rover_tools
- Paquetes: rover_diagnostics, rover_calibration
- Herramientas de diagnóstico y calibración

### Documentación
- Guías de instalación y hardware setup
- Quick start guide
- Documentación de arquitectura
- READMEs en cada workspace

### Scripts
- setup.sh: Instalación automática de dependencias
- build_all.sh: Compilación de todos los workspaces
- test_all.sh: Testing automatizado
- sync_docs.sh: Sincronización con página web

### Configuración
- Parámetros centralizados en rover_params.yaml
- Todas las dimensiones del robot
- Puertos de hardware
- Configuración de sensores

## Migrado desde Rover_old

### Parámetros Preservados
- wheel_radius: 0.075m
- wheelbase: 0.180m
- track_width: 0.250m
- encoder_cpr: 4096
- Configuración de RoboClaw
- Configuración de PX4
- Configuración de IBUS

### Mejoras sobre Rover_old
- Arquitectura modular (8 workspaces vs monolítico)
- Scripts de automatización
- Documentación completa
- CI/CD configurado
- Mejor organización de código
- Integración con Nav2
- Rutas relativas para portabilidad

---

## Versiones Futuras

### [1.0.0] - TBD
- Primera release estable
- Todos los drivers funcionando
- Nav2 completamente integrado
- Documentación completa
- Tests automatizados

### [0.1.0] - TBD
- Alpha release
- Funcionalidad básica
- Teleoperación funcional
- Drivers básicos

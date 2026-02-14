# Rover - Monorepo Profesional ROS2

Estructura completada el 14 de febrero de 2026.

## Estructura Creada

```
Rover/
├── .github/
│   └── workflows/
│       └── ci.yml                  # CI/CD con GitHub Actions
│
├── .vscode/
│   ├── tasks.json                  # Tasks de VSCode
│   └── settings.json               # Configuración de VSCode
│
├── docs/                           # Documentación completa
│   ├── index.md                    # Página principal
│   ├── getting-started/
│   │   ├── installation.md         # Guía de instalación
│   │   ├── hardware-setup.md       # Setup de hardware
│   │   └── quickstart.md           # Inicio rápido
│   ├── architecture/
│   │   └── overview.md             # Arquitectura del sistema
│   ├── guides/                     # Guías de uso
│   ├── api/                        # Documentación de API
│   ├── troubleshooting/            # Solución de problemas
│   └── assets/                     # Imágenes y recursos
│
├── scripts/
│   ├── setup.sh                    # Instalación automática
│   ├── build_all.sh                # Compilar todos los workspaces
│   ├── test_all.sh                 # Ejecutar todos los tests
│   └── sync_docs.sh                # Sincronizar docs con web
│
├── tools/                          # Herramientas de desarrollo
│
├── rover_core/                     # Workspace 1: Core
│   ├── src/
│   │   ├── rover_msgs/             # Mensajes personalizados
│   │   └── rover_interfaces/       # Servicios y acciones
│   └── README.md
│
├── rover_drivers/                  # Workspace 2: Drivers
│   ├── src/
│   │   ├── roboclaw_driver/        # Driver RoboClaw
│   │   ├── ibus_driver/            # Driver IBUS
│   │   └── px4_bridge/             # Bridge PX4
│   └── README.md
│
├── rover_control/                  # Workspace 3: Control
│   ├── src/
│   │   ├── rover_base_controller/  # Controlador de base
│   │   └── rover_teleop/           # Teleoperación
│   └── README.md
│
├── rover_navigation/               # Workspace 4: Navegación
│   ├── src/
│   │   ├── rover_nav2_bringup/     # Configuración Nav2
│   │   │   ├── launch/
│   │   │   ├── params/
│   │   │   └── maps/
│   │   └── rover_localization/     # Localización
│   └── README.md
│
├── rover_perception/               # Workspace 5: Percepción
│   ├── src/                        # (Listo para sensores)
│   └── README.md
│
├── rover_simulation/               # Workspace 6: Simulación
│   ├── src/
│   │   ├── rover_description/      # URDF y modelos
│   │   │   ├── urdf/
│   │   │   ├── meshes/
│   │   │   ├── config/
│   │   │   └── launch/
│   │   └── rover_gazebo/           # Gazebo
│   │       ├── launch/
│   │       ├── worlds/
│   │       └── models/
│   └── README.md
│
├── rover_bringup/                  # Workspace 7: Integración
│   ├── src/
│   │   ├── rover_launch/           # Launch files principales
│   │   │   ├── launch/
│   │   │   └── config/
│   │   └── rover_config/           # Configuración centralizada
│   │       └── config/
│   │           └── rover_params.yaml  # Todos los parámetros
│   └── README.md
│
├── rover_tools/                    # Workspace 8: Tools
│   ├── src/
│   │   ├── rover_diagnostics/      # Diagnóstico
│   │   └── rover_calibration/      # Calibración
│   └── README.md
│
├── .gitignore                      # Git ignore
├── LICENSE                         # Licencia MIT
├── README.md                       # README principal
├── CHANGELOG.md                    # Historial de cambios
├── CONTRIBUTING.md                 # Guía de contribución
└── rover.repos                     # Archivo vcstool (futuro)
```

## Archivos de Configuración Creados

### rover_params.yaml
Contiene todos los parámetros del rover:
- Dimensiones físicas (wheel_radius: 0.075m, wheelbase: 0.180m, track_width: 0.250m)
- Configuración de hardware (puertos, baudrates)
- Parámetros de sensores
- Configuración de navegación
- Topics y frames
- Encoders (CPR: 4096)
- Velocidades máximas

### Scripts de Automatización
- `setup.sh`: Instala todas las dependencias automáticamente
- `build_all.sh`: Compila los 8 workspaces en orden de dependencias
- `test_all.sh`: Ejecuta tests en todos los workspaces
- `sync_docs.sh`: Sincroniza docs/ con la página web

### CI/CD
- GitHub Actions configurado para build y test automático
- Se ejecuta en push y pull requests

### VSCode
- Tasks configuradas para build, test y launch
- Settings para Python y ROS2
- Formateo automático

## Workspaces y Dependencias

```
rover_core (base)
    ↓
rover_drivers, rover_simulation
    ↓
rover_control
    ↓
rover_navigation, rover_perception, rover_tools
    ↓
rover_bringup (integración)
```

## Paquetes ROS2 Creados

Total: 13 paquetes

1. rover_msgs
2. rover_interfaces
3. roboclaw_driver
4. ibus_driver
5. px4_bridge
6. rover_base_controller
7. rover_teleop
8. rover_nav2_bringup
9. rover_localization
10. rover_description
11. rover_gazebo
12. rover_launch
13. rover_config
14. rover_diagnostics
15. rover_calibration

## Documentación

### Getting Started
- Installation.md: Guía completa de instalación
- Hardware-setup.md: Conexiones y configuración de hardware detallada
- Quickstart.md: Guía de inicio rápido

### Architecture
- Overview.md: Arquitectura completa del sistema con diagramas

### READMEs
- README.md principal del proyecto
- README.md en cada uno de los 8 workspaces
- Cada README documenta los paquetes y uso

## Parámetros Preservados del Rover_old

Todos los parámetros críticos fueron migrados:
- wheel_radius: 0.075m
- wheelbase: 0.180m
- track_width: 0.250m
- encoder_cpr: 4096
- Puertos: ttyACM0, ttyACM1, ttyACM2, ttyUSB0, ttyTHS0
- Baudrates: 115200
- Direcciones I2C: 128
- PID: P=1.0, I=0.5, D=0.25
- QPPS: 3300

## Próximos Pasos

1. Compilar todo:
   ```bash
   cd ~/Desktop/Rover
   ./scripts/setup.sh
   ./scripts/build_all.sh
   ```

2. Migrar código específico de Rover_old:
   - Implementación de drivers
   - Launch files específicos
   - Nodos de control

3. Testing:
   ```bash
   ./scripts/test_all.sh
   ```

4. Documentación:
   - Completar guías en docs/
   - Sincronizar con web: `./scripts/sync_docs.sh`

## Características

- Monorepo profesional modular
- 8 workspaces independientes
- Scripts de automatización
- CI/CD configurado
- Documentación completa
- Configuración centralizada
- Rutas relativas
- Sin Docker (portabilidad directa)
- Listo para Nav2
- Compatible con hardware existente

## Mejoras sobre Rover_old

1. Organización modular vs monolítica
2. Workspaces independientes compilables individualmente
3. Scripts de automatización
4. Documentación profesional completa
5. CI/CD desde el inicio
6. Configuración centralizada
7. Mejor estructura para desarrollo en equipo
8. Preparado para escalabilidad

## Sincronización con Página Web

```bash
./scripts/sync_docs.sh
```

Copia docs/ a: `/home/rover/Desktop/theblackrobotsfoundation.github.io/docs/rover/`

Luego publicar:
```bash
cd ~/Desktop/theblackrobotsfoundation.github.io
npm run build
git add docs/rover
git commit -m "Update rover documentation"
git push
```

---

**Proyecto:** Rover - Research Platform
**Organización:** The Black Robots Foundation
**Fecha:** Febrero 14, 2026
**Estado:** Estructura inicial completa

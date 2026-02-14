# Contribuyendo al Rover

Gracias por tu interés en contribuir al proyecto Rover. Este documento te guiará en el proceso.

## Código de Conducta

Se respetuoso con todos los contribuidores. Mantén el ambiente profesional y constructivo.

## Cómo Contribuir

### Reportar Bugs

1. Verifica que el bug no esté ya reportado en [Issues](https://github.com/TheBlackRobotsFoundation/Rover/issues)
2. Crea un nuevo issue con:
   - Título descriptivo
   - Pasos para reproducir
   - Comportamiento esperado vs actual
   - Sistema operativo y versión de ROS2
   - Logs relevantes

### Sugerir Mejoras

1. Abre un issue con label "enhancement"
2. Describe la mejora propuesta
3. Explica por qué sería útil
4. Propón una implementación si es posible

### Pull Requests

#### Proceso

1. Fork el repositorio
2. Crea una branch desde `develop`:
   ```bash
   git checkout -b feature/amazing-feature develop
   ```
3. Haz tus cambios
4. Asegúrate de que compile:
   ```bash
   ./scripts/build_all.sh
   ```
5. Ejecuta tests:
   ```bash
   ./scripts/test_all.sh
   ```
6. Commit con mensajes descriptivos:
   ```bash
   git commit -m "Add amazing feature"
   ```
7. Push a tu fork:
   ```bash
   git push origin feature/amazing-feature
   ```
8. Abre un Pull Request a la branch `develop`

#### Estilo de Código

**Python:**
- Sigue PEP 8
- Usa `black` para formateo
- Usa `flake8` para linting
- Docstrings en funciones públicas

**C++:**
- Sigue el estilo de ROS2
- Usa `clang-format`
- Documentación con Doxygen

**Launch files:**
- Python launch files (no XML)
- Comentarios explicativos
- Parámetros configurables

#### Commits

Formato de mensajes de commit:
```
<tipo>(<scope>): <descripción corta>

<descripción larga opcional>

<footer opcional>
```

Tipos:
- `feat`: Nueva funcionalidad
- `fix`: Corrección de bug
- `docs`: Cambios en documentación
- `style`: Formateo, punto y coma faltantes, etc.
- `refactor`: Refactorización de código
- `test`: Agregar o modificar tests
- `chore`: Mantenimiento, dependencias, etc.

Ejemplos:
```
feat(drivers): add support for new RoboClaw version

docs(readme): update installation instructions

fix(control): correct wheelbase calculation in odometry
```

## Estructura del Proyecto

### Agregar un Nuevo Paquete

1. Decide en qué workspace va:
   - `rover_core`: Mensajes/interfaces
   - `rover_drivers`: Drivers de hardware
   - `rover_control`: Control y teleoperación
   - `rover_navigation`: Navegación
   - `rover_perception`: Sensores
   - `rover_simulation`: Simulación
   - `rover_tools`: Herramientas

2. Crear el paquete:
   ```bash
   cd rover_<workspace>/src
   ros2 pkg create --build-type ament_python my_package
   ```

3. Agregar README al paquete

4. Actualizar dependencias en `package.xml`

5. Documentar en el README del workspace

### Agregar Documentación

1. Agregar archivo markdown en `docs/`
2. Seguir estructura existente
3. Incluir ejemplos de código
4. Agregar enlaces en `docs/index.md`

## Testing

### Escribir Tests

Cada paquete debe tener tests:

```python
# test/test_my_node.py
import unittest
import rclpy

class TestMyNode(unittest.TestCase):
    def test_something(self):
        # Test code
        pass
```

### Ejecutar Tests

```bash
# Test un workspace
cd rover_drivers
colcon test

# Test todo
./scripts/test_all.sh
```

## Documentación

- Actualiza READMEs relevantes
- Agrega docstrings en código
- Actualiza documentación en `docs/`
- Incluye ejemplos de uso
- Sincroniza con web si es necesario

## Review Process

1. CI debe pasar (build + tests)
2. Code review por maintainer
3. Cambios solicitados si es necesario
4. Merge a `develop`
5. Eventualmente merge a `main` en release

## Branches

- `main`: Producción estable
- `develop`: Desarrollo activo
- `feature/*`: Nuevas funcionalidades
- `fix/*`: Correcciones
- `docs/*`: Documentación

## Releases

1. Todo se desarrolla en `develop`
2. Cuando está listo para release:
   - Merge `develop` → `main`
   - Tag con versión semántica: `v1.0.0`
   - Actualizar CHANGELOG.md

## Contacto

- Issues: [GitHub Issues](https://github.com/TheBlackRobotsFoundation/Rover/issues)
- Web: [TheBlackRobotsFoundation](https://theblackrobotsfoundation.github.io)

Gracias por contribuir!

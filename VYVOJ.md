# Rychlý start pro vývoj

Úplný popis je v [README](README.md#getting-started), tohle je zkrácená verze.

Máš dvě prostředí: **robot_control** pro ovladač robota a **vision** pro kameru
a neuronové sítě. Obě vidí stejný repozitář, ale každé má vlastní knihovny
i výsledky sestavení.

## 1. Otevři prostředí

Na hostiteli otevři repozitář ve VS Code, přes `F1` spusť **Dev Containers:
Reopen in Container** a vyber jedno z nich. Vision potřebuje NVIDIA GPU
a NVIDIA Container Toolkit.

VS Code kontejner připraví a sestaví jeho ROS balíčky. Terminály v tom okně pak
běží uvnitř kontejneru. Sám o sobě kontejner nespustí nic.

Pro souběžnou práci otevři repozitář ve druhém okně a vyber druhé prostředí.
Zavření okna kontejnery nevypíná; zastavíš je z terminálu hostitele:

```bash
docker compose -f docker-compose.dev.yml --profile vision stop
```

## 2. Spusť uzel

V prostředí **robot_control**:

```bash
ros2 launch robot_control robot_control.launch.py
```

Controller čeká ve stavu `unconfigured`. Připojení k robotu aktivuješ z druhého
terminálu:

```bash
ros2 lifecycle set /robot_controller configure
ros2 lifecycle set /robot_controller activate
```

V prostředí **vision** nejdřív na hostiteli stáhni modely přes `git lfs pull`:

```bash
ros2 launch tvarometr_inference vision.launch.py device:=cuda:0 use_camera:=false
```

## 3. Upravuj kód

Uprav Python soubor, zastav uzel přes `Ctrl+C` a spusť `ros2 launch` znovu.
Zdrojáky se ukládají přímo do repozitáře na hostiteli.

Po změně ROS zpráv, akcí, entry pointů nebo launch/config souborů sestav
balíčky znovu a načti výsledek:

```bash
colcon build --symlink-install --packages-select robot_control_msgs robot_control
source /opt/colcon_ws/install/setup.bash
```

Vision staví `tvarometr_interfaces tvarometr_inference`.

Po změně knihoven nebo Dockerfile použij **Dev Containers: Rebuild Container**.

## Dva různé buildy

| Operace | Co připravuje | Kdy ji potřebuješ |
| --- | --- | --- |
| Docker build | Image se systémem, ROS a knihovnami. | Poprvé a po změně závislostí nebo Dockerfile. |
| Colcon build | Tvoje ROS balíčky: rozhraní, spustitelné příkazy, instalaci. | Automaticky při vytvoření kontejneru, pak ručně po změně rozhraní. |

## Kde jsou soubory

```text
/workspace/              repozitář sdílený s hostitelem
/opt/colcon_ws/build/    pracovní soubory Colconu v kontejneru
/opt/colcon_ws/install/  sestavené ROS balíčky v kontejneru
/opt/colcon_ws/log/      záznamy sestavení v kontejneru
```

Cesta `/opt/colcon_ws` se na hostitele nemountuje, takže build ani install se
ti do repozitáře nikdy nepropíšou. Přežijí zastavení a spuštění kontejneru, ale
při **Rebuild Container** zmizí — každý rebuild tak začíná načisto.

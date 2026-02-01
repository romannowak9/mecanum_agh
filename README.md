# ROS 2 + Gazebo Harmonic docker

Repozytorium zawiera kod do wykorzystania przez studentów w celu realizacji zajęć z Systemów i Algorytmów Percepcji w Pojazdach Autonomicznych (SiAPwPA) w semestrze zimowym 2024/2025.

## Wymagania

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Start

Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.

> [!TIP]
> Dla osób korzystających z Windowsa oraz WSL 2 przygotowano `Dockerfile.windows` oraz `compose.windows.yaml`. 

Po zalogowaniu się do dockera będzie on działał w sposób podobny do uruchamiania ROS na komputerze hosta. Wszystkie aplikacje GUI będą korzystać z domyślnego menedżera okien hosta, będziesz mieć również dostęp do wszystkich urządzeń na hoście, a także akceleracji GPU.
Docker ma preinstalowany [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html) i większość potrzebnych zależności oraz symulator [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/).

## Pierwsze uruchomienie

Dla osób, które nie miały doczynienia ze środowiskiem ROS 2 + Gazebo, zachęcam do przerobienia tutorialu: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/). Pozwoli to zaznajomić się z tym środowiskiem i tworzyć w przyszłości zaawansowane symulacje.

Następnie pomocne będzie odpowiednia kontrola robotami w środowisku symulacyjnym, na dobry start proszę zaznajomić się z repozytorium: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

Na sam koniec pewnym podsumowaniem, a także praktycznym podejściem do tematu jest dostarczony od [Husariona](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) tutorial dla ich kilku robotów.

> [!IMPORTANT] 
Należy pamiętać, aby po zbudowaniu wywołać komendę lub pracować w nowym terminalu:
>
> ```bash
> source ~/.bashrc
> ```
>
> W tym pliku dodane są już dwie ważne ścieżki:
>
> ```bash
> /opt/ros/$ROS_DISTRO/setup.bash
> /home/developer/ros2_ws/install/setup.bash
> ```

## Przykład
1. Zbuduj obszar roboczy wraz z simple_example package.  
2. Uruchom launcha `example.launch.py`, pokazujący, w jaki sposób należy połączyć Gazebo z ROS 2, aby możliwa była wzajemna komunikacja.


## Dodatkowe materiały
* [Getting Started](docs/getting_started.md)
* [ROS 2 Command Cheat Sheet](docs/cheatsheet.md)
* [ROS 2 Example packages in Python](docs/example.md)
* [Bridge communication between ROS and Gazebo](docs/ros_gz_bridge.md)

## Dodatkowe informacje
W celu uruchomienia projektu bez wpisywania wielu komend, należy uruchomić następujące polecenia:
```bash
cd .. # Zmiania folderu do ~/ros2_ws
./launch.sh
```
W tym momencie skrypt upewnia się, że wszystkie ścieżki znajdują się w PATH, czy projekt jest zbudowany i uruchamia wszystkie widoki.


# Opis projektu

## Opis zadanisa - cele

- Wdrożenie symulacji oraz dostosowanie jej parametrów w celu opracowania testowego modelu algorytmu sterowania pojazdem autonomicznym mecanum.
- Implementacja algorytmu wizyjnego oraz sterującego prędkością i zwrotem pojazdu w symulacji.
- Przetestowanie lidaru w symulacji oraz zaimplementowanie algorytmu omijania przeszkód.
- Przetestowanie rzeczywistego lidaru Livox.
- Przeniesienie aplikacji na platformę mecanum wyposażoną w NVidia Jetson Xavier.

## Wyznaczanie punktu zadanego na podstawie obrazu

- Z obrazu kamery wykrywany jest żółty pas na podstawie progowania koloru w przestrzeni HSV.
- Spośród wykrytych konturów wybierany jest największy poprawny czworokąt, który nie dotyka krawędzi obrazu i ma wystarczające pole
- Na podstawie wierzchołków czworokąta wyznaczane są środki lewego i prawego boku pasa. Punkt zadany w 2D to środek odcinka łączącego te dwa punkty (środek pasa w obrazie).
- Z szerokości pasa w pikselach oraz znanej rzeczywistej szerokości pasa obliczana jest odległość Z od kamery
- Punkt 2D jest rzutowany do przestrzeni 3D układu kamery z użyciem macierzy parametrów wewnętrznych kamery

## MasterController - opis algorytmu sterowania pojazdem

- Z gamepada określany jest tryb pracy pojazdu autonomicznego (MANUAL lub AUTO w przypadku braku pada).
- W trybie MANUAL odczytywane są osie joysticka, a na ich podstawie wyznaczane są prędkości liniowe i kątowe pojazdu.
- Wciśnięcie przycisku STOP zeruje prędkości i zatrzymuje pojazd.
- W trybie AUTO publikowana jest prędkość wyznaczona przez moduł AutomaticController, chyba że aktywne jest unikanie przeszkód.
- Jeśli aktywne jest unikanie przeszkód, zamiast prędkości automatycznej publikowana jest prędkość korekcyjna otrzymana z modułu unikania.

## AutomaticController - opis algorytmu sterowania pojazdem

- Z modułu kamery odbierany jest punkt zadany w układzie obrazu, który określa boczne odchylenie pojazdu względem środka pasa.
- Błąd boczny jest przetwarzany przez regulator PID lub Purse Persuit, który wyznacza wartość skrętu pojazdu.
- Z modułu LiDAR odbierana jest informacja o obecności przeszkód z przodu, z tyłu oraz po bokach pojazdu.
- Jeśli przeszkoda znajduje się z przodu, pojazd zatrzymuje ruch do przodu i wykonuje skręt w stronę wolnej przestrzeni.
- Jeśli przeszkoda znajduje się tylko z lewej lub tylko z prawej strony, regulator PID/PP jest korygowany tak, aby wymusić minimalny skręt w stronę wolnej - przestrzeni.
- Z danych IMU integrowane jest przyspieszenie w czasie, co pozwala oszacować aktualną prędkość pojazdu w osi X.
- Wygenerowana komenda ruchu jest zapisywana jako najnowsza prędkość automatyczna i przekazywana do MasterController w trybie AUTO.

## Procedura omijania przeszkody

- Z danych LaserScan wyznaczane są minimalne odległości do przeszkód
- Jeśli minimalna odległość z przodu spadnie poniżej progu bezpieczeństwa, uznawane jest to za wykrycie przeszkody.
- Po wykryciu przeszkody wybierana jest strona omijania (lewa lub prawa) – ta, po której przeszkoda jest bliżej.
- Robot przechodzi w fazę skrętu w miejscu, aż przeszkoda znajdzie się z boku robota
- Następnie aktywowany jest tryb podążania wzdłuż ściany:
    - mierzona jest odległość do ściany po wybranej stronie,
    - liczony jest błąd względem zadanej odległości,
    - prędkość kątowa wyznaczana jest regulatorem P.
- Gdy przeszkoda przestaje blokować tor jazdy i kamera ponownie widzi punkt zadany, omijanie zostaje zakończone.







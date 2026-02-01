# Projekt z przedmiotu Systemy i Algorytmy Percepcji w Pojazdach Autonomicznych

Repozytorium zawiera kod studentów realizujący projekt w ramach przedmiotu Systemy i Algorytmy Percepcji w Pojazdach Autonomicznych w roku akademickim 2025/2026.

## Wymagania

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Start

### Symulator

Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.
W celu uruchomienia projektu bez wpisywania wielu komend, należy uruchomić następujące polecenia:
```bash
cd .. # Zmiania folderu do ~/ros2_ws
./launch.sh
```
W tym momencie skrypt upewnia się, że wszystkie ścieżki znajdują się w PATH, czy projekt jest zbudowany i uruchamia wszystkie widoki.

### Uruchomienie na robocie

W celu uruchomienia projektu na robocie Mecanum poleca się użyć Nvidia Jetson Xavier NX o numerze "1", gdyż w folderze użytkownika znajduje się tam gotowy kod z zbudowanym Dockerem. Niestety nie przeniesiono wszystkich zmian, które wprowadzono w celu uruchomienia robota na repozytorium. Na repozytorium znajduje się zmodyfikowany obraz kontenera, który działa na procesorze o architekturze ARM. Zmiany w kodzie do działania na robocie można sprawdzić [w tym repozytorium](https://github.com/romannowak9/mecanum_agh/tree/arduino_input).

Przed uruchomieniem obrazu zaleca się podpięcie do komputera Jetson pada do gier, a także wprowadzenie następująch poleceń w terminalu tego komuptera
```bash
sudo chmod 666 /dev/input/event*
sudo chmod 666 /dev/input/js*
```
Jest to obejście problemu, przez który Docker nie wykrywał poprawnie pada. Uruchomienie środowiska na robocie wygląda podobnie do przypadku, w którym stosujemy symulator, jednak najpierw korzystając z VS Code, należy podpiąć się do komputera Jetson przy pomocy SSH i wejść do folderu użytkownika. W nim czeka nas folder *mecanum_agh*, w którym wykonujemy resztę kroków.

W momencie uruchomienia programu robot znajduje się w trybie sterowania autonomicznego, to jest będzie poszukiwał pasów na ziemii.

#### Sterowanie przy pomocy pada

* Przycisk **B** (na padzie od XBoxa) - zmiana trybu sterowania. Wciskanie tego przycisku na przemian uruchamia tryb autonomiczny i manualny,
* Przycisk **X** (na padzie od XBoxa) - "hamulec", przytrzymanie tego przycisku odcina sterowania napędem, przez co robot wyhamuje, działa tylko w ręcznym trybie sterownia,
* Lewa gałka analogowa - sterowanie robotem, działa tylko w trybie ręcznego sterowania.


## Opis projektu

### Opis zadanisa - cele

- Wdrożenie symulacji oraz dostosowanie jej parametrów w celu opracowania testowego modelu algorytmu sterowania pojazdem autonomicznym mecanum.
- Implementacja algorytmu wizyjnego oraz sterującego prędkością i zwrotem pojazdu w symulacji.
- Przetestowanie lidaru w symulacji oraz zaimplementowanie algorytmu omijania przeszkód.
- Przetestowanie rzeczywistego lidaru Livox.
- Przeniesienie aplikacji na platformę mecanum wyposażoną w NVidia Jetson Xavier.

### Wyznaczanie punktu zadanego na podstawie obrazu

- Z obrazu kamery wykrywany jest żółty pas na podstawie progowania koloru w przestrzeni HSV.
- Spośród wykrytych konturów wybierany jest największy poprawny czworokąt, który nie dotyka krawędzi obrazu i ma wystarczające pole
- Na podstawie wierzchołków czworokąta wyznaczane są środki lewego i prawego boku pasa. Punkt zadany w 2D to środek odcinka łączącego te dwa punkty (środek pasa w obrazie).
- Z szerokości pasa w pikselach oraz znanej rzeczywistej szerokości pasa obliczana jest odległość Z od kamery
- Punkt 2D jest rzutowany do przestrzeni 3D układu kamery z użyciem macierzy parametrów wewnętrznych kamery

### MasterController - opis algorytmu sterowania pojazdem

- Z gamepada określany jest tryb pracy pojazdu autonomicznego (MANUAL lub AUTO w przypadku braku pada).
- W trybie MANUAL odczytywane są osie joysticka, a na ich podstawie wyznaczane są prędkości liniowe i kątowe pojazdu.
- Wciśnięcie przycisku STOP zeruje prędkości i zatrzymuje pojazd.
- W trybie AUTO publikowana jest prędkość wyznaczona przez moduł AutomaticController, chyba że aktywne jest unikanie przeszkód.
- Jeśli aktywne jest unikanie przeszkód, zamiast prędkości automatycznej publikowana jest prędkość korekcyjna otrzymana z modułu unikania.

### AutomaticController - opis algorytmu sterowania pojazdem

- Z modułu kamery odbierany jest punkt zadany w układzie obrazu, który określa boczne odchylenie pojazdu względem środka pasa.
- Błąd boczny jest przetwarzany przez regulator PID lub Purse Persuit, który wyznacza wartość skrętu pojazdu.
- Z modułu LiDAR odbierana jest informacja o obecności przeszkód z przodu, z tyłu oraz po bokach pojazdu.
- Jeśli przeszkoda znajduje się z przodu, pojazd zatrzymuje ruch do przodu i wykonuje skręt w stronę wolnej przestrzeni.
- Jeśli przeszkoda znajduje się tylko z lewej lub tylko z prawej strony, regulator PID/PP jest korygowany tak, aby wymusić minimalny skręt w stronę wolnej - przestrzeni.
- Z danych IMU integrowane jest przyspieszenie w czasie, co pozwala oszacować aktualną prędkość pojazdu w osi X.
- Wygenerowana komenda ruchu jest zapisywana jako najnowsza prędkość automatyczna i przekazywana do MasterController w trybie AUTO.

### Procedura omijania przeszkody

- Z danych LaserScan wyznaczane są minimalne odległości do przeszkód
- Jeśli minimalna odległość z przodu spadnie poniżej progu bezpieczeństwa, uznawane jest to za wykrycie przeszkody.
- Po wykryciu przeszkody wybierana jest strona omijania (lewa lub prawa) – ta, po której przeszkoda jest bliżej.
- Robot przechodzi w fazę skrętu w miejscu, aż przeszkoda znajdzie się z boku robota
- Następnie aktywowany jest tryb podążania wzdłuż ściany:
    - mierzona jest odległość do ściany po wybranej stronie,
    - liczony jest błąd względem zadanej odległości,
    - prędkość kątowa wyznaczana jest regulatorem P.
- Gdy przeszkoda przestaje blokować tor jazdy i kamera ponownie widzi punkt zadany, omijanie zostaje zakończone.







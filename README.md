# Optymalizacja Planowania Ścieżki: RRT* z Adaptacyjnym Próbkowaniem (AIW-LDIW)

Projekt badawczy analizujący wpływ adaptacyjnych strategii doboru wag (Adaptive Inertia Weight) na wydajność i jakość ścieżek w algorytmach **RRT** (Rapidly-exploring Random Tree) oraz **RRT***.

## 🎯 Cel Projektu

Rozwiązanie kompromisu między **szybkością znajdowania ścieżki** (zbieżność) a jej **jakością** (koszt/długość). Projekt porównuje klasyczne podejście ze stałym parametrem celowania (*Fixed Goal Bias*) z nowatorskim podejściem hybrydowym (*AIW-LDIW*). W dynamicznych środowiskach czas reakcji jest priorytetem. Dlatego kluczowym celem projektu jest maksymalizacja szybkości zbieżności algorytmu oraz minimalizacja kosztu obliczeniowego.

## 🚀 Kluczowe Funkcje

* **Algorytmy:** Implementacja RRT oraz RRT* (z optymalizacją *Rewire*).
* **Strategie Próbkowania:**
    * **Fixed:** Stałe prawdopodobieństwo celowania w cel.
    * **AIW (Adaptive Inertia Weight):** Dynamiczna zmiana wagi w zależności od wskaźnika sukcesu ($P_s$).
    * **Hybrid (AIW-LDIW):** Połączenie adaptacji do sukcesu z liniowym spadkiem wagi w czasie.
* **Informed Sampling:** Ograniczenie przestrzeni losowania do elipsy po znalezieniu wstępnej ścieżki.
* **Benchmark:** Moduł do automatycznego testowania wydajności na różnych mapach.

## 📂 Struktura Plików

* `rrt.py` - Główna biblioteka z implementacją klas `Tree`, `treeNode` oraz logiką algorytmów (funkcja `rrt_solver`).
* `testy.py` - Skrypt do masowego uruchamiania testów (benchmarku) i zapisywania wyników do CSV/PKL.
* `rrt_not.ipynb` - Jupyter Notebook do wizualizacji pojedynczych ścieżek oraz generowania wykresów statystycznych.
* `images/` - Folder zawierający mapy testowe (np. `mapa1.png`, `mapa2.png`) oraz przykladowe wyniki wyszukiwania sciezek.

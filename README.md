------------------------------
## MyClockESP32 – Dokumentacja Optymalizacji (Wersja Ultra-Stable)
Projekt został zaktualizowany z myślą o pracy ciągłej (24/7), eliminując typowe problemy platformy ESP32 związane z zarządzaniem pamięcią oraz kolizjami zasobów.
## 1. Architektura Wielordzeniowa (FreeRTOS)
Wykorzystano pełną moc dwurdzeniowego procesora ESP32 poprzez separację zadań (Tasks):

* Rdzeń 0 (PRO_CPU): Obsługa stosu sieciowego, Wi-Fi oraz portalu konfiguracyjnego AutoConnect. Dzięki temu intensywny ruch sieciowy nie wpływa na precyzję odliczania czasu.
* Rdzeń 1 (APP_CPU): Logika zegara, obsługa przerwań wyświetlacza (ISR), odczyt czujników oraz loop(). Gwarantuje to płynność animacji i multipleksowania LED.

## 2. Optymalizacja Pamięci (Stability)

* Pamięć FLASH (PROGMEM): Cały interfejs webowy (HTML/CSS/JS) został przeniesiony do pamięci programu. Zwalnia to ok. 15-20 KB pamięci RAM (Heap), zapobiegając restartom spowodowanym fragmentacją sterty przy odświeżaniu panelu /config.
* Eliminacja klasy String: W kluczowych punktach (endpoint /status) zastąpiono klasę String bezpiecznymi buforami char i funkcją snprintf.

## 3. Niezawodność Odczytów (DS18B20)

* Mechanizm Retry: Wprowadzono potrójną próbę odczytu temperatury w przypadku błędu -127 (brak połączenia).
* Walidacja danych: System odrzuca błędne odczyty, a w przypadku trwałej awarii czujnika automatycznie wygasza sekcję temperatury, zapobiegając wyświetlaniu nieaktualnych danych.
* Asynchroniczność: Odczyt temperatury nie blokuje procesora dzięki wykorzystaniu vTaskDelay zamiast standardowych opóźnień bibliotecznych.

## 4. Wyświetlacz i Interfejs (UX)

* Hardware Timer ISR: Multipleksowanie wyświetlacza 7-segmentowego odbywa się w przerwaniu sprzętowym, co zapewnia zerowe migotanie niezależnie od obciążenia procesora.
* Predykcja czasu w UI: Panel webowy wykorzystuje lokalną predykcję czasu w JavaScript, co eliminuje opóźnienia sieciowe ("lag sekundy") w przeglądarce.
* Histereza LDR: Algorytm EMA (Exponential Moving Average) z progiem czułości zapobiega migotaniu jasności matrycy przy granicznych warunkach oświetleniowych.

## 5. Bezpieczeństwo Systemowe

* Task Watchdog Timer (WDT): Monitorowanie głównej pętli programu oraz krytycznych zadań. W przypadku zawieszenia logiki, system automatycznie wykonuje restart.
* Safe Reboot: Procedura restartu (np. po aktualizacji OTA) realizowana jest z opóźnieniem w osobnym wątku, co pozwala serwerowi na poprawne domknięcie sesji HTTP.
* Diagnostyka NTP: Dodano rejestrację czasu ostatniej udanej synchronizacji z serwerami czasu (GUM/NTP Pool).

------------------------------

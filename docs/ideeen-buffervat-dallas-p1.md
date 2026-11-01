# Prompt: plan van aanpak buffervat-installatie (P1-pomp, Dallas-sensoren, vloerbeveiliging)

> Status: verkennende fase. Opgesteld op 2026-11-01 en bijgewerkt na de sync met upstream
> Jordi1990/openamber (0.8.0+25, branch `sync-upstream-0.8`). Gebruik deze tekst als startprompt
> om met Claude een plan van aanpak te maken. Wat hieronder "bestaat al" staat, is geverifieerd in
> de code én op het display (firmware 0.8.0).

---

## Prompt

Ik wil een plan van aanpak maken voor uitbreidingen van OpenAmber voor mijn installatie.
Maak eerst een plan (nog niet implementeren). Verifieer aannames in de code en stel vragen
waar hardwarekeuzes van mij afhangen. **Flash het display niet zonder mijn uitdrukkelijke
toestemming.** Ik wil geen uitval van het scherm, en een eerdere lokale build viel na OTA terug
naar 0.8.0 (oorzaak onbekend).

### Mijn installatie

- Itho Daalderop **HP-S 95**, aangestuurd door OpenAmber op een **Waveshare 5" display**
  (ESP32-S3, `src/openamber-waveshare-display.yaml`). Het display staat op
  http://192.168.178.51/ en draait firmware 0.8.0.
- **Buffervat van 100 liter** tussen warmtepomp en afgifte.
- **Tc-sensor zit in het buffervat.** De regeling werkt op de "regeltemperatuur"
  (`GetControlTemperature()` in `src/openamber/controllers/heat_cool_controller.h`). Die staat nu
  op Tc, dus de buffertemperatuur wordt naar de stooklijn geregeld.
- **Primair circuit:** pomp **P0** (intern, PWM, met eigen PID op ΔT) pompt van de warmtepomp
  door het buffervat.
- **Secundair circuit:** een **externe pomp** van het buffervat naar de **vloerverwarming**. Die
  loopt nu los op warmtevraag en wordt **niet** door de warmtepomp gestuurd.
- **4 DS18B20-sensoren (Dallas):** aanvoer en retour van/naar het buffervat, en aanvoer en retour
  van de vloerverwarming.
- **Tv1 en Tv2 lezen 0,0 °C**, dus daar is geen voeler op de binnenunit aangesloten.
- De stooklijn staat op de standaardwaarden (25/27/28/30/35 °C bij +15/+10/+5/0/−10). Ik test
  die eerst een paar dagen.

### Bestaat al in 0.8.0 (alleen nog instellen of testen)

| Functie | Waar | Huidige instelling op het display |
|---|---|---|
| **P1-pomp aansturen**: P1 draait bij pompvraag, stopt bij ontdooien en zonder vraag | schakelaar "Pomp P1 aanwezig" (`src/openamber/core/switches.yaml:4`), logica `StartPumpP1IfNeeded()` / `StopPumpP1IfNeeded()` in `heat_cool_controller.h` | UIT |
| **Tv1 als regeltemperatuur** in plaats van Tc | select "Regeltemperatuur bron" (`src/openamber/core/selects.yaml:133`). Let op: bij actieve mengventielen wordt altijd Tc gebruikt. | CV-aanvoer (Tc) |
| **PID verwarmen instelbaar** (Kp, Ki, Kd, deadband) | `src/openamber/core/numbers.yaml` (`pid_heat_kp` enz.), ook op het scherm onder instellingen | 0.60 / 0.0020 / 0.70 / 1.0 °C |
| **Start/stop-delta verwarmen** | `compressor_start_delta_heating` / `compressor_stop_delta_heating` | standaard |
| **Backup-heater graadminuten** | `backup_heater_degmin_threshold` | standaard |
| **Zone 2 en mengventielen** | stooklijn zone 2 en mengventiel min/max (`numbers.yaml`), `mixing_valve_actuator.h` | niet in gebruik |
| **Vertraagd uitschakelen thermostaat** | upstream #259 | nakijken |

### Ideeën om uit te werken

1. **Externe vloerpomp op P1 aansluiten** *(software bestaat al)*
   - Uitzoeken: welke klemmen P1 zijn op de binnenunit, of het relais spanning levert of
     potentiaalvrij is, en het maximale vermogen tegenover mijn pomp.
   - Daarna "Pomp P1 aanwezig" aanzetten en controleren of het gedrag past: P1 stopt tijdens
     ontdooien. Is dat gewenst met een buffervat ertussen? Moet er naloop komen?

2. **Dallas-sensoren op het Waveshare-scherm aansluiten** *(nieuw)*
   - De meeste GPIO's zijn bezet door het RGB-scherm. Bezet: GPIO43/44 (RS485), GPIO8/9 (I²C),
     GPIO4 (touch).
   - Voorkeur: de **sensor/AD-header** (vermoedelijk **GPIO6**, nog verifiëren op het bordschema).
     Geïsoleerde DI/DO, CAN en RS485 zijn niet geschikt voor 1-wire.
   - Alternatief: een DS2482-100 1-wire-brug op I²C (ESPHome-ondersteuning nakijken) of de
     waarden via Home Assistant (alleen voor weergave, niet voor regeling of beveiliging).
   - Alle 4 sensoren op één bus, pull-up 4,7 kΩ (of ca. 2,2 kΩ bij lange kabels).
   - ESPHome: `one_wire` (platform gpio) + `dallas_temp` in `src/openamber-waveshare-display.yaml`.
     De adressen haal je uit de opstartlog.

3. **Vloeraanvoer als regeltemperatuur**
   - Optie A: een NTC op de **Tv1-ingang** van de binnenunit (welk NTC-type? zie de
     Amber-handleiding) en "Regeltemperatuur bron" op Tv1 zetten. Dat vraagt geen
     firmwarewijziging.
   - Optie B: de Dallas-vloeraanvoer als extra bron toevoegen aan `GetControlTemperature()` en de
     select. Dat is wel een firmwarewijziging.
   - Afwegen: regelen op het buffervat (Tc) of op de vloeraanvoer.

4. **Weergave op het scherm** *(nieuw)*
   - De vier temperaturen als tegels op het homescherm (`src/openamber/ui/page_home.yaml`, met
     bindings in `src/openamber/ui/bindings/`).
   - Afgeleide waarden: ΔT van de vloerverwarming (aanvoer − retour) en het verschil tussen Tc
     (buffer) en de vloeraanvoer (menging/verlies in het vat).

5. **Overtemperatuurbeveiliging vloerverwarming** *(nieuw)*
   - Stop de compressor en/of P1 als de vloeraanvoer boven een instelbare grens komt
     (bijvoorbeeld 45 °C).
   - Bedenken wat er moet gebeuren als een sensor uitvalt (fail-safe).

6. **Regeling afstemmen op het buffervat** *(instellingen bestaan al)*
   - Na de testperiode met de standaard stooklijn: PID (Kp/Ki/Kd/deadband), start/stop-delta's en
     de graadminutengrens van de backup-heater afstemmen op een traag buffervat van 100 L.
   - Letten op: comfort, setpoint tegenover Tc, starts per uur, ongewenst inschakelen van de
     backup-heater.

7. **Modelherkenning HP-S 95** *(lopend)*
   - Het display toont "Onbekend model". Register 3257 (`model_identification_1`) geeft een
     andere waarde dan de bekende codes (140/150/15/145). Outdoor EEPROM V0708,120148,
     analytics device-ID 9931d9.
   - Op `sync-upstream-0.8` toont de firmware de ruwe code ("Onbekend model (code X)"), maar
     geflasht draaide die build niet (rollback). Alternatief zonder flashen: Jordi1990 vragen welke
     `a1`-waarde binnenkomt voor device-ID 9931d9.
   - Daarna de code toevoegen aan de mapping (`src/openamber/core/text_sensors.yaml`,
     `src/openamber/ui/bindings/script_ui_bindings.yaml`, `src/openamber/core/sensors.yaml`
     voor het pompvermogen) en een PR/issue upstream indienen.

8. **README-correctie** *(klein)*
   - `README.md` regels 100 en 113 noemen Tv1/Tv2 nog "Outdoor coil temperature". Dat moet
     "Zone 1/2 flow temperature" zijn (zo staat het ook in de foutcodetabel F21/F22). Kan als PR
     upstream.

### Open vragen voor mij

- Welke klemmen zijn P1 op de binnenunit, en past mijn vloerpomp op dat relais?
- Is P1 stoppen tijdens ontdooien gewenst? Wil ik naloop?
- Welke GPIO zit op de sensor/AD-header van mijn Waveshare-bordversie?
- Wil ik regelen op Tc (buffer) of op de vloeraanvoer (Tv1-NTC of Dallas)?
- Moet de vloerbeveiliging de compressor stoppen, P1, of beide? Welke grenswaarde?

# Svetla u automobilu

## 1. Opis projekta

Simulacija sistema za upravljanje svjetlima u automobilu korišćenjem FreeRTOS-a. Sistem prima podatke sa senzora, obrađuje ih i upravlja svjetlima u automatskom ili manuelnom režimu. Za razmjenu podataka koriste se redovi (*queues*), a za sinhronizaciju semafori i softverski tajmeri.

## 2. Pokretanje periferija iz Command Prompt-a

Periferije se pokreću zasebno iz njihovih izvršnih (`.exe`) fajlova.

### Opšti postupak

1. Otvoriti **Command Prompt**.
2. Komandom `cd /d` preći u direktorijum u kojem se nalazi izvršni fajl. Primjer:

```bat
cd /d "PUTANJA_DO_DIREKTORIJUMA"
```

3. Pokrenuti odgovarajući `.exe` fajl i unijeti traženi argument.

### 2.1. Seg7Mux – 7-segmentni displej

Pri pokretanju se zadaje broj cifara displeja. U ovom projektu koristi se displej sa **9 cifara**, pa se unosi:

```text
9
```

### 2.2. LED_Bars – LED bar

Pri pokretanju se zadaju oznake ulaznog i izlaznog stuba, kao i boja:

- **malo slovo** označava ulazni stub;
- **veliko slovo** označava izlazni stub;
- zadaje se i boja LED-ova prema opcijama periferije.

Primjer korišćen u projektu:

```text
rR
```

`r` označava ulazni, a `R` izlazni stub.

### 2.3. UniCom

Pri pokretanju se zadaje broj komunikacionog kanala koji se želi otvoriti.

U projektu se koriste:

| Kanal | Namjena |
|---|---|
| 0 | Senzor spoljašnje osvijetljenosti |
| 1 | Senzor vrata i periodične poruke sistema |
| 2 | Prijem komandi sa računara i potvrde komandi |

Pokrenuti UniCom periferiju za svaki potreban kanal i unijeti odgovarajući broj kanala.

> **Napomena:** Tačan naziv `.exe` fajla i njegova puna putanja zavise od direktorijuma u kojem se nalazi simulator. Uputstvo iznad navodi argumente koje treba zadati; puni naziv komande dopuniti stvarnim nazivom izvršnog fajla.

Periferije treba pokrenuti prije testiranja aplikacije.

## 3. FreeRTOS zadaci

- `SensorLightReceive_Task` – prijem spoljašnje osvijetljenosti.
- `SensorDoorReceive_Task` – prijem stanja vrata.
- `SensorTrigger_Task` – periodično pokretanje očitavanja senzora.
- `PCReceive_Task` – prijem komandi sa računara.
- `PCSend_Task` – slanje informacija računaru.
- `DataProcessing_Task` – obrada podataka i određivanje stanja sistema.
- `LCDDisplay_Task` – priprema podataka za 7-segmentni displej i praćenje minimuma/maksimuma.
- `LEDBar_Task` – upravljanje LED barovima.

## 4. Senzori i mjerenja

Očitavanje senzora pokreće se svakih **200 ms**.

- Opseg spoljašnje osvijetljenosti: **0–1000**.
- Za automatsko upravljanje koristi se prosjek posljednjih **10 mjerenja**.

## 5. Komunikacija sa računarom

Komande se primaju preko **UniCom kanala 2**. Svaka komanda završava se znakom **CR** (decimalna vrijednost **13**).

Podržane komande:

```text
PRAG<vrednost>
MANUELNO
AUTOMATSKI
```

Primjer:

```text
PRAG500
```

Komandom `PRAG` podešava se prag osvijetljenosti za automatski režim. Nakon komande `MANUELNO` ili `AUTOMATSKI`, sistem šalje potvrdu:

```text
OK
```

Svake **2 sekunde** sistem preko **kanala 1** šalje prosječnu osvijetljenost i trenutni režim rada.

## 6. Manuelni režim

U manuelnom režimu ulazne LED direktno upravljaju odgovarajućim izlazima.

| Ulazna LED (odozdo) | Funkcija | Izlazna LED (odozdo) |
|---:|---|---:|
| 1 | Dnevna svjetla (DRL) | 8 |
| 2 | Kratka svjetla | 7 |
| 3 | Duga svjetla | 6 |
| 4 | Lijevi pokazivač pravca | 5 |
| 5 | Desni pokazivač pravca | 4 |
| 7 | Svjetlo u kabini | Posljednja izlazna LED |

Lijevi i desni pokazivači pravca trepere periodično, sa periodom od **500 ms**.

## 7. Automatski režim

Dnevna i kratka svjetla određuju se na osnovu prosječne osvijetljenosti i podešenog praga:

- prosjek **veći od praga**: DRL uključen, kratka svjetla isključena;
- prosjek **manji od praga**: DRL isključen, kratka svjetla uključena.

Kada osvijetljenost poraste iznad praga, kratka svjetla ostaju uključena još **5 sekundi**, a zatim se isključuju.

U automatskom režimu LED bar može upravljati samo:
- dugim svjetlima;
- lijevim pokazivačem pravca;
- desnim pokazivačem pravca.

DRL i kratka svjetla u ovom režimu kontrolišu se automatski, a ne odgovarajućim ulaznim LED-ovima.

### 7.1. Svjetlo u kabini

Stanje vrata određuje stanje kabinskog svjetla:

- vrata otvorena → kabinsko svjetlo uključeno;
- vrata zatvorena → kabinsko svjetlo isključeno.

Stanje kabinskog svjetla prikazuje se na posljednjoj izlaznoj LED.

## 8. 7-segmentni displej

Koristi se **Seg7Mux sa 9 cifara**:

- **4 cifre** – trenutna osvijetljenost;
- **1 cifra** – režim rada (`0` = automatski, `1` = manuelni);
- **4 cifre** – minimalna ili maksimalna izmjerena osvijetljenost od pokretanja sistema.

MIN i MAX predstavljaju stvarno zabilježene vrijednosti, a ne granice dozvoljenog opsega senzora.

Gornja ulazna LED bira šta se prikazuje:
- LED isključena → MIN;
- LED uključena → MAX.

Period osvježavanja displeja podešen je na **1500 ms**.

## 9. Uputstvo za testiranje

1. Iz Command Prompt-a pokrenuti Seg7Mux, LED_Bars i potrebne UniCom kanale.
2. Pokrenuti aplikaciju iz razvojnog okruženja.
3. Mijenjati osvijetljenost i provjeriti prosjek posljednjih 10 mjerenja, MIN/MAX i prikaz na displeju.
4. Poslati `PRAG500`, zatim `AUTOMATSKI`; mijenjati osvijetljenost iznad i ispod praga i provjeriti DRL, kratka svjetla i odlaganje od 5 sekundi.
5. Poslati `MANUELNO`; aktivirati pojedinačne ulazne LED-ove i provjeriti mapiranje iz tabele, uključujući treperenje pokazivača pravca.
6. Promijeniti stanje vrata i provjeriti da se kabinsko svjetlo uključuje pri otvorenim, a isključuje pri zatvorenim vratima.
7. Provjeriti prijem komandi na kanalu 2, odgovor `OK` i periodične poruke na kanalu 1.

## 10. MISRA analiza

MISRA analiza sprovodi se nad fajlom **`main_application.c`**, prema pravilima obrađenim na vježbama 5 i 6.

U konačnu dokumentaciju unijeti:
- naziv alata i način pokretanja provjere;
- obuhvaćena pravila;
- ispravljene nepravilnosti;
- konačan rezultat i eventualna opravdana izuzeća.

Rezultat treba popuniti nakon završnog pokretanja MISRA provjere.

## 11. GitHub obaveze

Obavezni dio predaje:
- najmanje jedan **Issue**;
- najmanje jedan **Pull Request**;
- popunjen `README.md` sa uputstvom za pokretanje i testiranje;
- MISRA analiza.

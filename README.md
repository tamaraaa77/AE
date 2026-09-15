# Svetla u automobilu

## Opis projekta

Projekat predstavlja simulaciju sistema za upravljanje svjetlima u automobilu realizovanog korišćenjem FreeRTOS-a. Sistem prikuplja podatke sa senzora, obrađuje ih i na osnovu izabranog režima upravlja odgovarajućim svjetlima.

Sistem podržava dva režima rada:

- **AUTOMATSKI režim**
- **MANUELNI režim**

Podaci između zadataka prenose se pomoću redova (queues), dok se za sinhronizaciju koriste semafori.

---

## FreeRTOS zadaci

U projektu su realizovani sljedeći zadaci:

- **SensorLightReceive_Task** – prijem podatka o spoljašnjoj osvijetljenosti.
- **SensorDoorReceive_Task** – prijem podatka o stanju vrata.
- **SensorTrigger_Task** – periodično pokretanje očitavanja senzora.
- **PCReceive_Task** – prijem komandi sa računara.
- **PCSend_Task** – slanje informacija računaru.
- **DataProcessing_Task** – obrada podataka sa senzora i određivanje trenutnog stanja sistema.
- **LCDDisplay_Task** – prikaz trenutne osvijetljenosti, režima rada i minimalne/maksimalne izmjerene vrijednosti.
- **LEDBar_Task** – upravljanje ulaznim i izlaznim LED barovima.

---

## Senzori

Podaci sa senzora očitavaju se svakih **200 ms**.

- **Kanal 0** – senzor spoljašnje osvijetljenosti.
- **Kanal 1** – senzor stanja vrata.

Opseg vrijednosti osvijetljenosti je **0–1000**.

Za određivanje trenutnog nivoa osvijetljenosti koristi se prosjek posljednjih **10 mjerenja**.

---

## Komunikacija sa računarom

Komande se primaju preko **UniCom kanala 2**. Svaka komanda se završava znakom **CR (decimalna vrijednost 13)**.

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

Komandom `PRAG` podešava se prag osvijetljenosti koji se koristi u automatskom režimu.

Nakon komandi `MANUELNO` i `AUTOMATSKI`, sistem šalje odgovor:

```text
OK
```

Svake **2 sekunde** sistem na **kanalu 1** prikazuje prosječnu osvijetljenost i trenutni režim rada.

---

# Manuelni režim

U manuelnom režimu korisnik direktno upravlja svjetlima pomoću ulaznog LED bara.

Mapiranje ulaznih i izlaznih LED-ova je:

| Ulazna LED | Funkcija | Izlazna LED |
|---|---|---|
| 1. odozdo | Dnevna svjetla (DRL) | 8. odozdo |
| 2. odozdo | Kratka svjetla | 7. odozdo |
| 3. odozdo | Duga svjetla | 6. odozdo |
| 4. odozdo | Lijevi pokazivač pravca | 5. odozdo |
| 5. odozdo | Desni pokazivač pravca | 4. odozdo |
| 7. odozdo | Svjetlo u kabini | posljednja izlazna LED |

Odgovarajuće izlazno svjetlo uključuje se kada je aktivna pripadajuća ulazna LED.

Lijevi i desni pokazivač pravca trepere periodično, sa periodom od **500 ms**.

---

# Automatski režim

U automatskom režimu uključivanje dnevnih i kratkih svjetala zavisi od prosječne spoljašnje osvijetljenosti i podešenog praga.

- Ako je **prosječna osvijetljenost veća od praga**:
  - DRL je uključen.
  - Kratka svjetla su isključena.

- Ako je **prosječna osvijetljenost manja od praga**:
  - DRL je isključen.
  - Kratka svjetla su uključena.

Kada osvijetljenost poraste iznad podešenog praga, kratka svjetla se **ne isključuju odmah**, već ostaju uključena još **5 sekundi**.

U automatskom režimu LED bar se može koristiti samo za:

- duga svjetla,
- lijevi pokazivač pravca,
- desni pokazivač pravca.

Dnevna i kratka svjetla u ovom režimu kontrolišu se automatski na osnovu osvijetljenosti.

### Svjetlo u kabini

Stanje vrata utiče na svjetlo u kabini:

- **vrata otvorena → svjetlo u kabini uključeno**
- **vrata zatvorena → svjetlo u kabini isključeno**

Stanje kabinskog svjetla prikazuje se na posljednjoj izlaznoj LED.

---

# 7-segmentni displej

Za prikaz podataka koristi se **Seg7Mux** sa 9 cifara.

Displej prikazuje:

- **4 cifre** – trenutna osvijetljenost,
- **1 cifra** – režim rada,
- **4 cifre** – minimalna ili maksimalna izmjerena osvijetljenost.

Oznake režima:

- `0` – AUTOMATSKI
- `1` – MANUELNI

Minimalna i maksimalna vrijednost predstavljaju stvarno izmjerene vrijednosti od pokretanja sistema, a ne granice dozvoljenog opsega senzora.

Gornja ulazna LED određuje šta se prikazuje:

- **LED OFF** → minimalna vrijednost,
- **LED ON** → maksimalna vrijednost.

Displej se osvježava prema zadatom periodu od **1500 ms**.

---

# Primjer rada

Ako su izmjerene vrijednosti osvijetljenosti:

```text
500
300
700
```

tada su:

```text
MIN = 300
MAX = 700
```

Ako je trenutna vrijednost 700, a sistem je u automatskom režimu:

```text
Trenutna osvijetljenost = 0700
Režim = 0
MIN = 0300
MAX = 0700
```

U zavisnosti od stanja gornje ulazne LED, na displeju se prikazuje minimalna ili maksimalna vrijednost.

---

# Testiranje

### 1. Pokretanje sistema

Pokrenuti aplikaciju i provjeriti da se zadaci pravilno izvršavaju.

### 2. Test senzora osvijetljenosti

Unijeti više različitih vrijednosti osvijetljenosti i provjeriti:

- računanje prosjeka posljednjih 10 mjerenja,
- određivanje minimalne vrijednosti,
- određivanje maksimalne vrijednosti,
- prikaz vrijednosti na 7-segmentnom displeju.

### 3. Test automatskog režima

Postaviti prag, na primjer:

```text
PRAG500
```

Zatim mijenjati osvijetljenost iznad i ispod vrijednosti 500 i provjeriti automatsko uključivanje DRL i kratkih svjetala.

Provjeriti i odlaganje od **5 sekundi** pri prelasku sa kratkih svjetala na DRL.

### 4. Test manuelnog režima

Poslati:

```text
MANUELNO
```

Zatim aktivirati pojedinačne ulazne LED-ove i provjeriti da se uključuju odgovarajuće izlazne LED.

Posebno provjeriti:

- DRL,
- kratka svjetla,
- duga svjetla,
- lijevi pokazivač,
- desni pokazivač,
- kabinsko svjetlo.

### 5. Test automatskog režima

Poslati:

```text
AUTOMATSKI
```

Provjeriti da se DRL i kratka svjetla automatski upravljaju na osnovu osvijetljenosti, dok se duga svjetla i pokazivači mogu upravljati LED barom.

### 6. Test vrata

Promijeniti stanje vrata i provjeriti da se kabinsko svjetlo uključuje kada su vrata otvorena i isključuje kada su zatvorena.

### 7. Test komunikacije

Provjeriti:

- prijem komandi preko kanala 2,
- odgovor `OK`,
- periodično slanje prosječne osvijetljenosti i režima preko kanala 1.

---

## Struktura projekta

Glavna implementacija sistema nalazi se u:

```text
main_application.c
```

U projektu se koriste:

- FreeRTOS taskovi,
- queue mehanizmi,
- binarni semafori,
- softverski tajmeri,
- UniCom komunikacija,
- LED bar,
- Seg7Mux displej.

---

## GitHub

Za projekat je predviđeno:

- otvaranje **Issue-a**,
- rad kroz **Pull Request**,
- dodavanje ovog `README.md` fajla,
- dodavanje odgovarajućeg `.gitignore` fajla,
- dokumentovanje načina testiranja sistema.

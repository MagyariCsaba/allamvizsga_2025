# Bicikli felügyelő rendszer önvezető jármű technológiákhoz

Valós idejű kerékpárkövetési alkalmazás, amely GPS és IMU szenzor adatokat segítségével jelenít meg egy kerékpárt MQTT üzenetekből egy interaktív térképen Plotly és WebSocket technológiák használatával.

## 🚴‍♂️ Főbb Funkciók

- **Valós idejű követés**: Élő kerékpár pozíció és orientáció megjelenítés
- **Történeti útvonal elemzés**: Útvonalak lekérdezése és megjelenítése meghatározott időszakokból
- **Interaktív térképek**: Utcaszintű térképezés OpenStreetMap adatok használatával
- **Szenzor adatok integrációja**: GPS koordináták, gyorsulásmérő és giroszkóp adatok
- **Adatbázis tárolás**: Összes szenzor leolvasás tartós tárolása
- **WebSocket kommunikáció**: Valós idejű adatátvitel a webes felületre

## 📋 Rendszerkövetelmények

- Python 3.7+
- MySQL adatbázis
- MQTT broker (Mosquitto)
- Modern webböngésző WebSocket támogatással

## 🛠️ Telepítés

### 1. Projekt Letöltése és Környezet Beállítása

```bash
git clone <repository-url>
```

### 2. Szükséges Python Csomagok Telepítése

```bash
pip install sqlalchemy pymysql python-dotenv paho-mqtt osmnx plotly websockets numpy
```

### 3. Adatbázis Konfigurációja

Hozz létre egy `.env` fájlt a projekt gyökérkönyvtárában:

```env
DATABASE_URI=mysql+pymysql://felhasználónév:jelszó@host:port/adatbázis_név
```

Példa:
```env
DATABASE_URI=mysql+pymysql://root:jelszó@127.0.0.1:3306/jarmu_kovetes
```

### 4. MQTT Broker Beállítása

Mosquitto MQTT broker telepítése és indítása:

Hivatalos oldal: https://mosquitto.org/

## 📊 Adatbázis Séma

A rendszer automatikusan létrehozza a következő táblát:

```sql
CREATE TABLE mqtt_messages (
    id INTEGER PRIMARY KEY,
    timestamp DATETIME,
    gps_x FLOAT,
    gps_y FLOAT, 
    gps_z FLOAT,
    yaw FLOAT,
    roll FLOAT,
    pitch FLOAT,
    yaw2 FLOAT,
    roll2 FLOAT,
    pitch2 FLOAT
);
```

## 🚀 Használat

### 1. Alkalmazás Indítása

```bash
python main.py
```

Ez elvégzi a következőket:
- Adatbázis kapcsolat inicializálása
- WebSocket szerver indítása (8765-ös port)
- HTTP szerver indítása a webes felülethez (8000-es port)
- MQTT üzenetek figyelése az `eesTopic` témában
- Webes felület megnyitása az alapértelmezett böngészőben

### 2. MQTT Üzenet Formátum

A rendszer JSON üzeneteket vár az `eesTopic` témában a következő struktúrával:

```json
{
    "gpsPos": [szélesség, hosszúság, magasság],
    "imuAngles": [yaw, roll, pitch],
    "imu2Angles": [yaw2, roll2, pitch2]
}
```

### 3. Teszt Adatok Küldése

```bash
# Példa MQTT üzenet
mosquitto_pub -h localhost -t eesTopic -m '{
    "gpsPos": [46.5426, 24.5574, 350.0],
    "imuAngles": [0.1, 0.2, 0.3],
    "imuAngles2": [0.01, 0.02, 0.03]
}'
```

## 🖥️ Webes Felület

### Élő Követés Oldal (`http://localhost:8000`)
- Valós idejű kerékpár megjelenítés váz és kerék orientációval
- Utcatérkép réteg Marosvásárhely, Románia térségében
- Kapcsolat állapot jelző
- Automatikus térkép központosítás a jármű pozíciójára

### Útvonal Történet Oldal (`http://localhost:8000/route_history.html`)
- Történeti útvonalak lekérdezése dátum/idő tartomány alapján
- Interaktív útvonal megjelenítés
- Útvonal statisztikák (időtartam, pontok száma)
- Export lehetőségek

## 🏗️ Architektúra

### Backend Komponensek

- **`main.py`**: Alkalmazás belépési pont és szerver koordináció
- **`database_handler.py`**: SQLAlchemy ORM és adatbázis műveletek
- **`mqtt_client.py`**: MQTT üzenet feldolgozás és szenzor adatok kezelése
- **`map_drawer.py`**: OpenStreetMap integráció és kerékpár megjelenítés
- **`websocket_server.py`**: WebSocket szerver valós idejű kommunikációhoz
- **`route_handler.py`**: Történeti útvonal lekérdezés és adat feldolgozás

### Frontend Komponensek

- **`index.html`**: Élő követési felület
- **`route_history.html`**: Történeti útvonal elemzési felület
- **`map.js`**: Élő követés JavaScript logika
- **`route_history.js`**: Útvonal történet funkcionalitás
- **`styles.css`**: Reszponzív CSS stílus

## 🔧 Konfigurációs Beállítások

### Térkép Régió
A követési régió megváltoztatásához módosítsd a helyet a `main.py`-ban:

```python
map_drawer = MapDrawer("Te Városod, Országod")
```

### Frissítési Gyakoriság
Valós idejű frissítési gyakoriság beállítása az `mqtt_client.py`-ban:

```python
self.update_frequency = 1  # Minden N-edik üzenet feldolgozása
```

### Szerver Portok
Szerver portok módosítása a `main.py`-ban:

```python
HTTP_PORT = 8000  # Webes felület port
WS_PORT = 8765    # WebSocket port
```

## 📡 Adatfolyam

1. **Szenzor Adatok** → MQTT Broker → Python MQTT Kliens
2. **Adat Feldolgozás** → Adatbázis Tárolás + Valós idejű Számítás
3. **Megjelenítés** → WebSocket → Webes Felület
4. **Történeti Elemzés** → Adatbázis Lekérdezés → Útvonal Megjelenítés

## 🛡️ Hibakezelés

- Automatikus adatbázis újracsatlakozás kapcsolat megszakadás esetén
- WebSocket automatikus újracsatlakozás funkcionalitás
- MQTT üzenet validáció és hiba naplózás
- Elegáns leállás rendszer megszakítás esetén

## 📝 Fejlesztés

### Új Szenzor Típusok Hozzáadása
1. Frissítsd az `MQTTMessage` modellt a `database_handler.py`-ban
2. Módosítsd a `save_message` metódust az új mezők kezeléséhez
3. Frissítsd az MQTT üzenet feldolgozást az `mqtt_client.py`-ban

### Megjelenítés Testreszabása
- Módosítsd a kerékpár renderelést a `map_drawer.py`-ban
- Állítsd be a térkép stílust a frontend JavaScript fájlokban
- Frissítsd a CSS-t a felület testreszabásához

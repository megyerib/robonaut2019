# Verseny állapotgép + szükséges funkciók

---

## Indulás

✔️ Várakozás a start jelre majd továbblépés a következő állapotba

---

## Labirintus

* Labirintus felderítése, bejárása
  Közben: Átsoroló vonal megtalálása
  Bejárás végének meghatározása
* Átsoroló vonalra mozgás (irányhelyesen)

Teendők:
* Enkóder
* Inerciális szenzor
* Pozíció meghatározása
* Az előző vonal követése, a kanyarok felismerése mindkét irányból
* Átsoroló jelzés megatlálása
* Pozíció korrigálása ismert tereptárgyak alapján (ha pl. visszaérünk az első érzékelt kereszteződéshez)
* Irányított gráf tárolása (a tolatás több idő + lehet, hogy fordulni is kell)
  Az is lehet, hogy felesleges az irányított gráf
* Tolatás
* Max sebesség meghatározása (nem kell túlzásokba esni)

Ötlet:
Valami hasonló tömb tárolása, amiben mintákat lehet keresni
Csak egyenesben forsulhatnak elő minták?
(igazából mindegy, csak a hossztengely menti elmozdulást mérjük.)

```
*            *   |   *
*        *       |   * *
*     *          |   * *
*   *            |   *
*  *             |   * *
*  *             |   * *
```
A minták helyét lementve megkapjuk a gráfot.

Gráfbejátás: Egyszerű dijkstra?

```c
typedef struct
{
  POSITION p;
  VERTEX*  v[5];
  EDGEDIR  d[5];
}
VERTEX;

typedef enum
{
  rightFront,
  leftFront,
  rightRear,
  leftRear
}
EDGEDIR;
```

Vagy elég egy szomszédossági gráf + az élek kiindulási helye?

---

## Átsorolás

* Átsorolás a gyorsasági pályára
  Menj addíg, amíg középen nem lesz a vonal, aztán szabályozz -> Vonalra visszatalálás
* Safety car megtalálása

---

## Safety car

* Safety car követése
  Közben: pályán elfoglalt helyzet meghatározása
* Előzési pont megtalálása
* Előzés

Teendők:
* Sebességszabályzás
* ToF

---

## Gyorsasági

* 3 gyors kör
* Megállás

Teendők:
* Futómű beállítása a lejtőre/emelkedőre
* ...

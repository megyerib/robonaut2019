# Git policy

Valamit ide kellett tennem, hogy felcommit-olja a mappát. Kezdetnek 1-2 felmerült kérdés, javaslat:

* **Binárisok**at hogy (ne) commitoljunk?
* **HEX fájlok**: Nem árt, ha le vannak mentve, mert így fejlesztőkörnyezet nélkül is fel tudjuk programozni a mikrokontrollereket. Az eclipse projekteket érdemes úgy generálni, hogy a gyökérbe rakjon egy HEX-et és akkor .gitignore-ral le lehet tiltani az egész debug/release mappát.
* **Eclipse workspace**: Mint user specifikus dolog, nem szerencsés verziókövetni, ezért mindenki tartsa a git mappán kívül és importálja bele az stm32 mappás projekteket.

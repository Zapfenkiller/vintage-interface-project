## V. I. P. = **V**intage **I**nterface **P**roject

Ein altehrwürdiges fischertechnik Computing-Interface erhält Anschluss an den
I²C-Bus.

Die Eckdaten:

* I²C-Bus bis 400 kHz maximal
* I²C-Bus bis 100 kHz *ohne* Clock-Stretching
* Level-Shifter für den I²C-Bus eingebaut, Vcc = 1,8 V bis 5,0 V
* Bis zu 4 verschiedene V. I. P. im gleichen Bussegment adressierbar.
* Theoretisch beliebig viele Interfaces kaskadierbar
* I²C-Stecker entspricht 1:1 der I²C-Belegung des fischertechnik
[TX-Controllers 500995](https://ft-datenbank.de/details.php?ArticleVariantId=2f4d3bd4-b293-4f10-9b9d-c2faa8632409)

Was es über die alten Interfaces und den Adapter zu wissen gibt, ist bereits
als Mehrteiler in der ft:pedia veröffentlicht:

* Einführung und jede Menge Steckerbelegungen: [ft:pedia 2/2017, S. 63, V. I. P. - Teil 1](https://www.ftcommunity.de/ftpedia_ausgaben/ftpedia-2017-2.pdf)
* Stromversorgung und Motortreiber: [ft:pedia 3/2017, S. 57, V. I. P. - Teil 2](https://www.ftcommunity.de/ftpedia_ausgaben/ftpedia-2017-3.pdf)
* Eingänge und Watchdog: [ft:pedia 4/2017, S. 36, V. I. P. - Teil 3](https://www.ftcommunity.de/ftpedia_ausgaben/ftpedia-2017-4.pdf)

Hier im Repository sind alle Dokumente über die Hardware (Schaltplan und
Layout) und die Software (C-Sourcecode, Makefile, Hex-File und ein paar Files
für das AVR-Studio) zu finden.

Die Platine für den Adapter muss man sich selbst anfertigen.
Das Layout ist doppelseitig ausgeführt.
Eine Durchkontaktierung wird empfohlen.

Das Hex-File wird via AVR-Studio und STK500 in den angeschlossenen µC
programmiert.
Alternative Möglichkeiten sind natürlich ebenso verwendbar.
Dank des ISP-Steckers auf der Platine kann der µC direkt im Fertiggerät
geflasht werden.
Dieser ISP-Stecker ist 1:1 wie der 6-polige ISP-Stecker vom STK500 belegt.

An sich ist das Projekt abgeschlossen, evaluiert und die Software passt genau
zur Hardware.
Wer trotzdem selbst Hand anlegen möchte, kann dies gerne tun.
Die verwendete Toolchain (ist heutzutage auch alt, aber damit ist das Projekt
nun einmal entwickelt worden):

* µC-Referenz:   ATtiny2313 datasheet Rev. 2543I–AVR–04/06 (www.atmel.com)
* Compiler:      WinAVR-20100110 (http://www.sourceforge.net)
* Debugger:      AVR-Studio 4.18 (www.atmel.com)
* ISP:           Atmel STK500 (z. B. https://www.reichelt.de/Programmer-Entwicklungstools/AVR-STK-500/3/index.html?ACTION=3&LA=2&ARTICLE=34093&GROUPID=2969&artnr=AVR+STK+500&trstct=pol_7)
* PCB-CAD:       EAGLE 4.15 (http://www.cadsoft.de)

Gerade im ISP Bereich der Programmer gibt es erheblich günstigere Produkte
als das angegebene STK500.
Bitte im Bedarfsfall mal selbst beim Lieblingsbauteileshop mit dem Suchbegriff
"AVR programmer" stöbern.

Beim EAGLE kann möglicherweise auch eine modernere Version verwendet werden.
Ich habe das *nicht* ausprobiert.

Beim Compiler ändern sich von Version zu Version gerne mal die Definitionen
der Interruptbehandlung. 
Mit der angegebenen Version - und dem hiesigen makefile - klappt das auf
Anhieb.
Für andere Versionen ist möglicherweise massive Nacharbeit im C-Code zu
leisten.
Das Hex-File entspricht dem vorgestellten Sourcecode und kann sofort verwendet
werden.
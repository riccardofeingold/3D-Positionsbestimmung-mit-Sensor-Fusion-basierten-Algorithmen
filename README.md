# 3D-Positionsbestimmung mit Sensor Fusion-basierten Algorithmen

## Entwicklung eines Arduino-basierten Embedded-System zur Höhenmessung

In dieser Arbeit wird die Entwicklung eines Arduino-basierten Höhenmeters unter der Verwendung von Sensor-Fusion-Prinzipien und -Methoden dokumentiert und kommentiert. Dieses Embedded-System besitzt einen Barometer, ein GPS-Modul (Global Positioning System) und einen IMU-Sensor (Inertial Measurement Unit). Hinzu kommen noch Elemente der Bedienoberfläche wie etwa ein LC-Display und eine Tastatur. 

Das Ziel der Arbeit ist, ein Produkt vorzustellen, dass eine Höhengenauigkeit von ± 5 m erreicht. Dies wird mit Hilfe eines Map Matching-Algorithmus bewerkstelligt, der grundsätzlich dafür sorgt, dass fortlaufend eine Kalibrierung des Barometers als auch des GPS stattfindet (siehe Kapitel Map Matching).

Da aber für diese Anwendung eine genaue horizontale Positionsbestimmung Voraussetzung ist, werden auch dafür Lösungen vorgestellt – zentrale Rolle wird v.a. der «Kalman»-Filter (siehe Kapitel Kalman-Filter) haben, wobei nebst dem auch eine Alternative (siehe Kapitel Alternativ-Filter) präsentiert wird. Bei diesen Methoden wird der IMU-Sensor (siehe Kapitel IMU-Sensor) von grosser Bedeutung sein.

Sie finden das PDF der Maturaarebeit im Ordner Texte.

# Anleitung für die Verwendung des Höhenmesser sowie des Quellcodes:

Wenn Sie das Produkt vom Autor erhalten haben, können Sie es bereits direkt verwenden, sobald dieser an eine Stromversorgung angeschlossen wurde. Dabei ist es wichtig, ihn während des Aufstartens im regungslosen Zustand zu lassen, damit der IMU-Sensor des System kalibriert werden kann - d.h. einfach auf den Tisch liegen lassen bis die Anzeige "I'm ready!" auftaucht. Ebenfalls wichtig dabei ist, dass Sie das Produkt möglichst von elektronischen Geräten fernhalten.  

Falls Sie gerne Fixpunkte von ihrer Umgebung nehmen möchten, können Sie das, indem Sie die Ordner BHwMMaKF und libraries in Ihr Arduino Sketchbook einfügen. Dann starten Sie die Software von Arduino (für diesen Code wurde die Version 1.8.10 verwendet) und öffnen via Toolbar den Ordner BHwMMaKF. Jetzt sehen Sie den kompletten Code für das Produkt. Klicken Sie nun auf das Skript MapMatching.h und tragen Ihre Referenzpunkte, die Sie auf SwissMap ausgewählt haben, in das Array data[]. Dabei darauf achten, dass zuerst die Latitude und die Longitude kommt und dann die Höhe.

Wenn Sie auch den Datensatz für die Map Matching-Höhenmessung ändern möchten, dann können Sie dies auch gerne tun. Sie finden im Ordner SwissMapDaten/DatenSortierer das komplette Gitternetz der Schweiz - nebst dem existiert auch noch der Datensatz, welcher der Autor für seine Umgebung benutzt hat. Kopieren Sie einfach diejenigen Referenzpunkte, die Sie für Ihre Umgebung benötigen, in das Array dataset[], welcher sich im Skript HEIGHT_DATA.h befindet, oder starten Sie das Programm "SortData.exe" mittels des Terminals (Mac) oder der CommandLine (Windows) - beim Terminal geschieht dies mit ./SortData.exe - und geben Sie die vom Programm verlangten Daten an; ein paar Sekunden Geduld und Sie erhalten automatisch Ihre auf SwissMap ausgewählte Umgebung als Datensatz. 

Bei der Anwendung dieser Höhenmessung kann es durchaus der Fall sein, dass eine berechnete Höhe unvernüftig erscheinen wird. Dies ist auch beim Autor der Fall - der Grund konnte nicht herausgefunden werden. Der Autor wird dies demnächst behandelt haben.

Im Ordner Pythonista finden Sie noch die Implementation eines mehrdimensionalen Kalman-Filters. Damit Sie dieses Programm starten können, benötigen Sie die App Pythonista, welche Sie einfach über den AppStore oder GooglePlayStore herunterladen können (kostet um die 10 Fr.). Sobald Sie es gestartet haben, klicken Sie einmal auf Button und der Filter wird aktiviert. 

Im Ordner Messungen finden Sie alle gemachten Datensammlungen, die der Autor während der Maturaarbeitszeit gesammelt hat.

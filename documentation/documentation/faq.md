# Veelgestelde vragen (FAQ)

## Probleemoplossing

- **Seriële poort kan niet worden geopend**: controleer `port` en Linux-rechten (groep `dialout`).
- **Geen reactie van de grijper**: controleer bekabeling, baudrate en `slave_address`.
- **Build- of includefouten in de editor**: source je ROS- en workspace-setupbestanden zodat language tooling de afhankelijkheden ziet.

## Opmerkingen

- Bij het opstarten maakt de node verbinding en activeert de grijper.
- Bij het afsluiten deactiveert de node de driver en verbreekt de verbinding.
- Interne conversie zet de ruwe apparaatpositie (`0..255`) om naar modelpositie met `max_joint_position`.
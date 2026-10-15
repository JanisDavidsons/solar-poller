# ecoMAX860P3-C parameters, full read 2026-10-14

Read-only snapshot through ecoNET 300 (192.168.8.3): `rmParamsNames`, `rmParamsData`,
`rmParamsDescs`, `rmParamsEnums`, `rmParamsUnitsNames`. No write call was made.
Boiler state at read time: wood (182 = Režģis - Koksne), mode 3 Operation, flue gas
131 °C, blower 86 % (param 186 modulated by the boiler panel). Values are the
controller's at that moment; ranges are the ones ecoNET reports (some are derived
from other parameters, see the notes). Descriptions are the controller's own text
(Latvian UI). Machine-readable copy: `ecomax-params-2026-10-14.csv`.

## Notes that change how to read the table

- **36 Min. ventilat. jauda = 85 sets the floor of every blower-power parameter**
  (186, 155, 158, 34, 19, 28 all report min 85; 19 and 28 even show min > max).
  Description: it only narrows the range the user may set. Not verified by a write.
- **37 Min. vilkmes vent. jauda = 16 is the same floor for exhaust-fan parameters**
  (187, 8, 20, 29, 35, 156, 159 all report min 16).
- **182 Siltuma avots** is the fuel/heat-source selector: 0 Deglis - Briketes,
  1 Režģis - Koksne, 2 Rezerves katls. It reads 1 during a wood burn.
- **Wood burn-out per the controller's text:** 185 counts from the moment the flame
  brightness falls below 25 Liesmas noteikšana (5 %); after 185 min the controller
  tries to light the burner (with 183 = ON, pellets). The flame brightness is
  `tempOpticalSensor` in regParams (100 at read time); the poller does not publish it.
- **34 Vent. Jauda** (96) has the same name as 186 but is the pellet burner's
  blower in supervision (block 31-35). 186 is the wood (grate) blower.

## All parameters

| # | Name | Value | Range / options | Unit | Description |
|---|---|---|---|---|---|
| 0 | Maksimālā katla jauda | 16 | 16..23 | kW |  |
| 1 | Starpjauda | 15 | 6..15 | kW |  |
| 2 | Min. katla jauda | 5 | 2..14 | kW |  |
| 3 | Maksimālā jauda - ventilators | 55 | 55..55 | % | Ventilatora jauda kad deglis darbojas ar maksimālo jaudu. |
| 4 | Starpjauda - ventilators | 54 | 86..54 | % | Ventilatora jauda kad deglis darbojas ar starpjaudu. |
| 5 | Minimālā jauda - ventilators | 85 | 85..85 | % | Ventilatora jauda kad deglis darbojas ar minimālo jaudu. |
| 6 | Vilkmes maks.jauda | 34 | 30..100 | % | Vilkmes jauda pie maksimālās degļa jaudas |
| 7 | Starpjauda, vilkme | 29 | 25..33 | % | Vilkme pie degļa darbības pie starpjaudas |
| 8 | Min. jauda - vilkme | 24 | 16..28 | % | Vilkmes jada kad deglis darbojas ar minimālo jaudu |
| 9 | Starpjauda - histerēze H2 | 3 | 1..30 | °C | Temperatūras slieksnis pārejai starp degļa maksimālo jaudu un starpjaudu. |
| 10 | Minimālā jauda - histerēze H1 | 3 | 1..30 | °C | Temperatūras slieksnis pārejai starp degļa starpjaudu un minimālo jaudu. |
| 11 | Katla histerēze | 30 | 1..30 | °C | Ja katla temperatūra ir pazeminājusies zem iestatītās katla temperatūras par katla histerēzes vērtību, notiek automātiskā degļa aizdedzināšana. |
| 12 | Cikla laiks DARBA režīmā | 20 | 1..250 | sec. | Visa degvielas padeves cikla laiks DARBA režīmā. CIKLA LAIKS DARBA REŽĪMĀ = PADEVES LAIKS DARBS = PADEVĒJA BEZDARBĪBAS LAIKS. |
| 13 | Min. katla jauda FL | 0 | 0..100 | % | Minimālā jauda, ar kuru deglis var darboties Fuzzy Logic režīmā. |
| 14 | Max. katla jauda FL | 100 | 0..100 | % | Maksimālā jauda, ar kuru deglis var darboties  Fuzzy Logic režīmā. |
| 15 | Parametrs A FuzzyLogic | 5 | 2..8 |  | Parametrs, kas saistīts ar katla jaudas automātisko modulāciju, kad iepriekš iestatītā katla temperatūra ir stabilizējusies. Palielinot vērtību palielina katla jaudas palielināšanās ātrumu. Pārāk augsta vērtība var izraisīt nestabilu katla darbību. |
| 16 | Parametrs B FuzzyLogic | 40 | 20..120 |  | Parametrs, kas saistīts ar katla jaudas automātisko modulāciju, kad iepriekš iestatītā katla temperatūra ir stabilizējusies. Palielinot vērtību palielina katla jaudas palielināšanās ātrumu. Pārāk augsta vērtība var izraisīt nestabilu katla darbību. |
| 17 | Parametrs C FuzzyLogic | 15 | 0..75 |  | Parametrs, kas saistīts ar katla jaudas automātisko modulāciju, kad iepriekš iestatītā katla temperatūra ir stabilizējusies. Nav ieteicams mainīt šī parametra vērtību. |
| 18 | Vadības režīms | 1 (FuzzyLogic) | 0=Standarta / 1=FuzzyLogic / 2=Lambda FuzzyLogic | enum | STANDARD režīms ietver trīspakāpju degļa jaudas modulāciju. FUZZY LOGIC režīms ietver bezpakāpju degļa jaudas regulēšanu, balstoties uz PID algoritmu. |
| 19 | Aizdedz. ventilators 1 | 85 | 85..45 | % | Sākotnējā ventilatora jauda aizdedzināšanas laikā. Ja vērtība ir pārāk liela, aizdedzināšanas process var aizņemt ilgāku laiku vai neizdoties. |
| 20 | Vilkme pie aizdedzināš.mēģ. 1 | 28 | 16..100 | % | Izplūdes ventilatora aizdedzes jauda. Augsta vērtība var izraisīt neievainojumu. |
| 21 | Aizdedz. testa laiks | 10 | 10..240 | sec. | Laiks, lai pārbaudītu, vai deglis ir karsts. Viņš strādā tikai ventilators. Ja liesma ir pietiekami daudz spilgtumu uz pāreju no darba režīma bez režīmā Lekurin. |
| 22 | Aizdedzin. laiks | 6 | 1..20 | min. | Viena aizdedzināšanas mēģinājuma laiks. Pēc šī laika regulators vēl mēģina aizdedzināt katlu. Ja visi mēģinājumi ir neveiksmīgi, parādās trauksmes ziņojums: "Neveiksmīgs aizdedzināšanas mēģinājums". |
| 23 | Kurināmā deva | 100.0 | 10.0..2550.0 | g | Kurināmā svars pie aizdedzināšanas. Attiecas uz pirmo aizdedzināšanas mēģinājumu. Turpmākajos testos kurināmā deva ir mazāka. |
| 24 | Aizdedz-ja uzsilde | 40.0 | 2.0..500.0 | sec. | Aizdedzinātāja uzsildīšanas laiks pirms ventilatora ieslēgšanās. Nav jābūt pārāk garam, lai nesabojātu sildelementu. Pēc šī laika sildelements darbojas joprojām līdz liesmas noteikšanai. |
| 25 | Liesmas noteikšana | 5 | 1..100 | % | Liesmas noteikšanas slieksnis % gaismas, kuru sasniedzot, regulators atzīst, ka krāsns jau ir aizdedzināta. To arī izmanto, lai noteiktu degvielas trūkumu. |
| 26 | Darba laiks ar minimālo jaudu | 2 | 0..100 | min. | Darba laiks ar minimālo jaudu. |
| 27 | Kvēlošanas laiks | 50 | 1..200 | sec. | Kvēlošanas laiks pēc aizdedzināšanas. |
| 28 | Ventilatora darbība pēc aizdedzināšanas | 85 | 85..35 | % | Ventilatora darbība pēc aizdedzināšanas. |
| 29 | Vilkme pēc aizdedzināšanās | 26 | 16..100 | % |  |
| 30 | Ventilatora pulsācija | 0 (OFF) | 0=OFF / 1=ON | enum | Ventilatora pulsācija pie aizdedzināšanas |
| 31 | Gaidīšanas laiks | 0 | 0..60 | min. | Maksimālais regulatora GAIDĪŠANAS laiks. Pēc šī laika, no brīža, kad regulators pāriet uz GAIDĪŠANAS režīmu, notiek automātiskā degļa izdzēšana. GAIDĪŠANAS LAIKS = 0 režīms ir pilnīgi izslēgts. GAIDĪŠANAS LAIKS = 255 režīms ir izslēgts. |
| 32 | Katla jauda gaidīšanas režīmā | 2.0 | 0.2..51.0 | kW |  |
| 33 | Cikla laiks | 35 | 1..250 | sec. | Darbības cikla laiks gaidīšanas režīmā |
| 34 | Vent. Jauda | 96 | 85..100 | % | Ventilatora jauda gaidīšanas režīmā |
| 35 | Vilkmes vent. jauda | 22 | 16..100 | % | Vilkmes vent. jauda gaidīšanas laikā |
| 36 | Min. ventilat. jauda | 85 | 10..100 | % | Minimālā ventilatora jauda, kuru var iestatīt lietotājs. Izmanto tikai, lai samazinātu pieejamo ventilatora jaudas diapazonu. Šim parametram jābūt pēc iespējas mazākam - tādam, lai ventilators grieztos lēni un brīvi. |
| 37 | Min. vilkmes vent. jauda | 16 | 10..100 | % | Minimālā vilkmes jauda, ko lietotājs var iestatīt. To lieto tikai, lai ierobežotu pieejamo ventilatora jaudas diapazonu. Šim parametram jābūt pēc iespējas mazākam - tādam, lai ventilators grieztos lēni un brīvi. |
| 38 | Maksimālais izdzēšanas laiks | 12 | 1..60 | min. | Pēc šī laika aktivizējas PĀRTRAUKUMA režīms, lai gan liesmas sensors norāda liesmas klātbūtni. |
| 39 | Minimālais izdzēšanas laiks | 4 | 1..60 | min. | Izdzēšana ilgst vismaz tik ilgi, lai gan liesmas sensors norāda liesmas trūkumu. |
| 40 | Parametrs A LAMBDA | 10 | 1..80 |  | Ietekmē skābekļa daudzuma regulēšanas ātrumu izplūdes gāzēs līdz iestatītai vērtībai un  skābekļa daudzuma uzturēšanas stabilitāti izplūdes gāzēs. Nav ieteicams mainīt šo parametru. |
| 41 | Parametrs B LAMBDA | 100 | 10..200 |  | Ietekmē skābekļa daudzuma regulēšanas ātrumu izplūdes gāzēs līdz iestatītai vērtībai un  skābekļa daudzuma uzturēšanas stabilitāti izplūdes gāzēs. Nav ieteicams mainīt šo parametru. |
| 42 | Parametrs C LAMBDA | 0 | 0..75 |  | Ietekmē skābekļa daudzuma regulēšanas ātrumu izplūdes gāzēs līdz iestatītai vērtībai un  skābekļa daudzuma uzturēšanas stabilitāti izplūdes gāzēs. Nav ieteicams mainīt šo parametru. |
| 43 | Ventilatora korekcijas diapazons | 20 | 0..50 | % | Noteic pieļaujamo ventilatora jaudas variācijas diapazonu, izmantojot Lambda zondi. |
| 44 | Maksimālā jauda - skābeklis | 6.2 | 0.1..25.0 | % | Iestatītā skābekļa vērtība dūmgāzēs kad deglis darbojas ar maksimālo jaudu. |
| 45 | Starpjauda - skābeklis | 7.0 | 0.1..25.0 | % | Iestatītā skābekļa vērtība dūmgāzēs kad deglis darbojas ar starpjaudu. |
| 46 | Minimālā jauda - skābeklis | 9.0 | 0.1..25.0 | % | Iestatītā skābekļa vērtība dūmgāzēs kad deglis darbojas ar minimālo jaudu. |
| 47 | Ventilators režģa režīmā | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum |  |
| 48 | Padevēja veiktspēja | 11.4 | 0.2..50.0 | kg/h | Galvenā degvielas padevēja ātrums kg/h. |
| 49 | Degvielas kaloritāte | 4.8 | 0.1..25.0 | kWh/kg | Degvielas kaloritāte kWh/kg. Granulām apm. 4,9 kWh/kg. Oglēm apm. 6,5kWh/kg |
| 50 | Degv tvertnes izmērs | 300.0 | 2.0..500.0 | kg | Degvielas tvertnes tilpums degvielas līmeņa aprēķināšanai. Ievadot pareizu vērtību, lietotājam nav nepieciešams veikt degvielas līmeņa kalibrēšanu. Pēc degvielas līmeņa kalibrēšanas, regulators vairs neizmanto vērtību TVERTNES TILPUMS. |
| 51 | Rotācijas tīrīšanas cikls | 180 | 20..255 | sec. |  |
| 52 | Tīrīšanas intensitāte | 3 | 2..25 | % |  |
| 53 | Uzstādītā Katla temperatūra | 75 | 40..85 | °C | Iestatītā katla temperatūra kad laikapstākļu vadība ir izslēgta. Var tikt automātiski mainīta ar regulatoru, piem. kad iestatītā katla temperatūra ir zemāka par iestatīto SKŪ temperatūru. |
| 54 | Min. iestatāmā katla temp. | 40 | 30..80 | °C | Minimālā iestatītā katla temperatūra, kuru lietotājs var iestatīt lietotāja izvēlnē, un minimālā temperatūra, kuru regulators var automātiski iestatīt, piem. balstoties uz nakts samazinājumiem, laikapstākļu vadības parametriem, u.tml. |
| 55 | Maks. iestatāmā katla temp. | 85 | 30..90 | °C | Maksimālā iestatītā katla temperatūra, kuru lietotājs var iestatīt lietotāja izvēlnē, un maksimālā temperatūra, kuru regulators var automātiski iestatīt, piem. balstoties uz laikapstākļu vadības parametriem, u.tml. |
| 56 | Iestatītā KŪ temp. | 55 | 20..70 | °C | Temperatūra, līdz kurai tiks sasildīta sadzīves karstā ūdens tvertne. Uzmanību: pārāk augsta temperatūra var izraisīt applaucēšanos ar karstu ūdeni! |
| 57 | Min. iestatītā KŪ temp. | 20 | 20..55 | °C | Tas ir parametrs, ar kuru jūs varat ierobežot lietotājam iestatīt pārāk zemu uzstādīto karstā ūdens temperatūru. |
| 58 | Maks. iestatītā KŪ temp. | 70 | 25..80 | °C | Parametrs nosaka līdz kādai maksimālajai temperatūrai karstā ūdens tvertne tiks uzkarsēta. Pārāk augstas vērtības iestatīšana var radīt draudus lietotājiem iegūt apdegumus no karstā ūdens. |
| 59 | KŪ sūkņa režīms | 2 (Nav prioritāte) | 0=Izslēgts / 1=Prioritāte / 2=Nav prioritāte | enum | Ļauj izvēlēties sadzīves karstā ūdens darba režīmu. BEZ PRIORITĀTES - paralēla SKŪ un centrālapkures sūkņu darbība, PRIORITĀTE - darbojas tikai SKŪ sūknis, VASARA - tikai sadzīves karstā ūdens SKŪ sagatavošana (ēkas apsilde ir izslēgta). |
| 60 | KŪ boilera histerēze | 5 | 1..30 | °C | Šis parametrs nosaka SKŪ sensora izmērīto temperatūras samazinājumu attiecībā uz iestatīto SKŪ temperatūru, pēc kura tiks ieslēgts SKŪ sūknis (SKŪ = sadzīves karstais ūdens). |
| 61 | KŪ dezinfekcija | 0 (Izslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Ieslēdzot šo parametru, vienu reizi nedēļā, pirmdien plkst. 2:00. sadzīves karstais ūdens tiks sasildīts līdz 70˚C, lai dezinficētu SKŪ tvertni. Uzmanību: applaucēšanas risks ar karstu ūdeni! |
| 62 | KŪ avots | 1 (Akumulācijas tvertne) | 0=Apkures katls / 1=Akumulācijas tvertne | enum | Norāda KŪ sagatavošanas avotu |
| 63 | VASARAs režīms | 0 (Ziema) | 0=Ziema / 1=Vasara / 2=Auto / 3=Ziema / 4=Vasara | enum | VASARAs režīmā centrālapkure tiek izslēgta, uzturot SKŪ apsildi. Automātiskā režīma ieslēgšanās notiek, balstoties uz laikapstākļu temperatūras sensora rādījumiem. |
| 64 | VASARA režīma ieslēgšanas temp. | 16 | 5..30 | °C | Ārējā temperatūra, virs kuras tiek ieslēgts VASARAs režīms, ja automātiskā VASARAs režīma detekcija ir ieslēgta. |
| 65 | VASARA režīma izslēgšanas temp. | 10 | 1..15 | °C | Ārējā temperatūra, zem kuras tiek izslēgts VASARAs režīms, ja automātiskā VASARAs režīma detekcija ir ieslēgta. |
| 66 | Centrālā sūkņa ieslēgšanas temp. | 62 | 30..80 | °C | Šis parametrs nosaka temperatūru, kuru sasniedzot ieslēgsies CA katla sūknis. Tas aizsargā katlu pret rasošanu, kas rodas kad auksts ūdens atgriežas no instalācijas. |
| 67 | CA sūkņa bezdarbība SKŪ iekraušanas laikā | 0 | 0..99 | min. | Šis parametrs ļauj periodiski ieslēgt katla sūkni (CA), kad notiek iekraušana, ar SKŪ tvertnes prioritāti. Pēc šī laika CA sūknis aktivizē ieprogrammēto laiku - 30s. Lai izslēgtu šo funkciju, parametru iestatīt uz nulli. |
| 68 | Katla temp. palielinājums no SKŪ un maisītāja | 5 | 3..15 | °C | Šis parametrs nosaka, par cik grādiem jāpaaugstinās iestatītajai katla temperatūrai, lai piepildītu SKŪ tvertni, buferi un maisītāja loku. Iestatītās katla temperatūras paaugstināšana notiek tik tad, kad iestatītā katla temperatūra ir zemāka par pārējām iestatītājām temperatūrām. |
| 69 | KŪ darba pagarināšana | 3 | 0..99 | min. | KŪ darbības pagarinājums |
| 70 | Siltummainis | 1 (JĀ) | 0=NĒ / 1=JĀ | enum | Attiecas tikai uz hidrauliskajām instalācijām ar siltummaiņiem starp atvērto uz aizvērto sistēmu. Pieejamās opcijas: JĀ = katla sūknis darbojas īsā ciklā katls - siltummainis, nav atkarīgs no VASARAS funkcijas vai sadzīves karstā ūdens prioritātes, NĒ = katla sūknis darbojas normāli. |
| 71 | Pretaizsalšanas temperatūra | 5 | 3..25 | °C |  |
| 72 | Maisītāj Iestatītā 1 | 35 | 25..45 | °C | Regulējamajam apkures lokam iestatīta temperatūra, kad ir ieslēgta laikapstākļu vadība. Kad maisītāja apkures lokam ir ieslēgta laikapstākļu vadība, šo parametru regulators neņem vērā. |
| 73 | Maisītāj Iestatītā 2 | 40 | 20..55 | °C | Regulējamajam apkures lokam iestatīta temperatūra, kad ir ieslēgta laikapstākļu vadība. Kad maisītāja apkures lokam ir ieslēgta laikapstākļu vadība, šo parametru regulators neņem vērā. |
| 74 | Maisītāj Iestatītā 3 | 55 | 20..70 | °C | Regulējamajam apkures lokam iestatīta temperatūra, kad ir ieslēgta laikapstākļu vadība. Kad maisītāja apkures lokam ir ieslēgta laikapstākļu vadība, šo parametru regulators neņem vērā. |
| 75 | Maisītāj Iestatītā 4 | 40 | 20..55 | °C | Regulējamajam apkures lokam iestatīta temperatūra, kad ir ieslēgta laikapstākļu vadība. Kad maisītāja apkures lokam ir ieslēgta laikapstākļu vadība, šo parametru regulators neņem vērā. |
| 76 | Maisītāj Iestatītā 5 | 40 | 20..55 | °C | Regulējamajam apkures lokam iestatīta temperatūra, kad ir ieslēgta laikapstākļu vadība. Kad maisītāja apkures lokam ir ieslēgta laikapstākļu vadība, šo parametru regulators neņem vērā. |
| 77 | Min. maisītāja 1 | 25 | 20..90 | °C | Šis parametrs ļauj ierobežot lietotājam iespēju iestatīt pārāk zemu maisītāja loka temperatūru. Automātiskā iestatītās temperatūras maiņa, piem. laicīgs samazinājums, neizraisa  iestatītās temperatūras samazinājumu zem šajā parametrā iestatītās vērtības. |
| 78 | Min. maisītāja 2 | 20 | 20..90 | °C | Šis parametrs ļauj ierobežot lietotājam iespēju iestatīt pārāk zemu maisītāja loka temperatūru. Automātiskā iestatītās temperatūras maiņa, piem. laicīgs samazinājums, neizraisa  iestatītās temperatūras samazinājumu zem šajā parametrā iestatītās vērtības. |
| 79 | Min. maisītāja 3 | 20 | 20..90 | °C | Šis parametrs ļauj ierobežot lietotājam iespēju iestatīt pārāk zemu maisītāja loka temperatūru. Automātiskā iestatītās temperatūras maiņa, piem. laicīgs samazinājums, neizraisa  iestatītās temperatūras samazinājumu zem šajā parametrā iestatītās vērtības. |
| 80 | Min. maisītāja 4 | 20 | 20..90 | °C | Šis parametrs ļauj ierobežot lietotājam iespēju iestatīt pārāk zemu maisītāja loka temperatūru. Automātiskā iestatītās temperatūras maiņa, piem. laicīgs samazinājums, neizraisa  iestatītās temperatūras samazinājumu zem šajā parametrā iestatītās vērtības. |
| 81 | Min. maisītāja 5 | 20 | 20..90 | °C | Šis parametrs ļauj ierobežot lietotājam iespēju iestatīt pārāk zemu maisītāja loka temperatūru. Automātiskā iestatītās temperatūras maiņa, piem. laicīgs samazinājums, neizraisa  iestatītās temperatūras samazinājumu zem šajā parametrā iestatītās vērtības. |
| 82 | Max. maisītāja 1 | 45 | 20..90 | °C | Ierobežo lietotājam un regulatoram iespēju iestatīt pārāk augstu maisītāja temperatūru. MAISĪTĀJA VADĪBA = GRĪDA IESLĒGTA noteic maksimālo apkures loka temperatūru, virs kuras maisītāja sūknis tiks izslēgts. |
| 83 | Max. maisītāja 2 | 85 | 20..90 | °C | Ierobežo lietotājam un regulatoram iespēju iestatīt pārāk augstu maisītāja temperatūru. MAISĪTĀJA VADĪBA = GRĪDA IESLĒGTA noteic maksimālo apkures loka temperatūru, virs kuras maisītāja sūknis tiks izslēgts. |
| 84 | Max. maisītāja 3 | 70 | 20..90 | °C | Ierobežo lietotājam un regulatoram iespēju iestatīt pārāk augstu maisītāja temperatūru. MAISĪTĀJA VADĪBA = GRĪDA IESLĒGTA noteic maksimālo apkures loka temperatūru, virs kuras maisītāja sūknis tiks izslēgts. |
| 85 | Max. maisītāja 4 | 85 | 20..90 | °C | Ierobežo lietotājam un regulatoram iespēju iestatīt pārāk augstu maisītāja temperatūru. MAISĪTĀJA VADĪBA = GRĪDA IESLĒGTA noteic maksimālo apkures loka temperatūru, virs kuras maisītāja sūknis tiks izslēgts. |
| 86 | Max. maisītāja 5 | 85 | 20..90 | °C | Ierobežo lietotājam un regulatoram iespēju iestatīt pārāk augstu maisītāja temperatūru. MAISĪTĀJA VADĪBA = GRĪDA IESLĒGTA noteic maksimālo apkures loka temperatūru, virs kuras maisītāja sūknis tiks izslēgts. |
| 87 | Telpas termost. mais 1 | 10 | 0..30 | °C | Iestatītās temperatūras samazinājums maisītāja lokā pēc telpas termostata kontaktu atvēršanās (sasniedzot iestatīto telpas temperatūru). |
| 88 | Telpas termost. mais 2 | 0 | 0..30 | °C | Iestatītās temperatūras samazinājums maisītāja lokā pēc telpas termostata kontaktu atvēršanās (sasniedzot iestatīto telpas temperatūru). |
| 89 | Telpas termost. mais 3 | 30 | 0..30 | °C | Iestatītās temperatūras samazinājums maisītāja lokā pēc telpas termostata kontaktu atvēršanās (sasniedzot iestatīto telpas temperatūru). |
| 90 | Telpas termost. mais 4 | 0 | 0..30 | °C | Iestatītās temperatūras samazinājums maisītāja lokā pēc telpas termostata kontaktu atvēršanās (sasniedzot iestatīto telpas temperatūru). |
| 91 | Telpas termost. mais 5 | 0 | 0..30 | °C | Iestatītās temperatūras samazinājums maisītāja lokā pēc telpas termostata kontaktu atvēršanās (sasniedzot iestatīto telpas temperatūru). |
| 92 | Āra temp. maisītājs 1 | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Kad laikapstākļu vadība ir ieslēgta, iestatītā temperatūra tiek aprēķināta, balstoties uz sildīšanas līkni un laikapstākļu sensora norādīto temperatūru. |
| 93 | Āra temp. maisītājs 2 | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Kad laikapstākļu vadība ir ieslēgta, iestatītā temperatūra tiek aprēķināta, balstoties uz sildīšanas līkni un laikapstākļu sensora norādīto temperatūru. |
| 94 | Āra temp. maisītājs 3 | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Kad laikapstākļu vadība ir ieslēgta, iestatītā temperatūra tiek aprēķināta, balstoties uz sildīšanas līkni un laikapstākļu sensora norādīto temperatūru. |
| 95 | Āra temp. maisītājs 4 | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Kad laikapstākļu vadība ir ieslēgta, iestatītā temperatūra tiek aprēķināta, balstoties uz sildīšanas līkni un laikapstākļu sensora norādīto temperatūru. |
| 96 | Āra temp. maisītājs 5 | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Kad laikapstākļu vadība ir ieslēgta, iestatītā temperatūra tiek aprēķināta, balstoties uz sildīšanas līkni un laikapstākļu sensora norādīto temperatūru. |
| 97 | Apkures miks. Līkne 1 | 0.8 | 0.1..4.0 |  | Sildīšanas līkne atspoguļo ēkas siltuma rādītājus. Ieteicamais maisītāja loka iestatījums: 0,2 - 0,6 |
| 98 | Apkures miks. Līkne 2 | 0.8 | 0.1..4.0 |  | Sildīšanas līkne atspoguļo ēkas siltuma rādītājus. Ieteicamais maisītāja loka iestatījums: 0,2 - 0,6 |
| 99 | Apkures miks. Līkne 3 | 0.8 | 0.1..4.0 |  | Sildīšanas līkne atspoguļo ēkas siltuma rādītājus. Ieteicamais maisītāja loka iestatījums: 0,2 - 0,6 |
| 100 | Apkures miks. Līkne 4 | 0.8 | 0.1..4.0 |  | Sildīšanas līkne atspoguļo ēkas siltuma rādītājus. Ieteicamais maisītāja loka iestatījums: 0,2 - 0,6 |
| 101 | Apkures miks. Līkne 5 | 0.8 | 0.1..4.0 |  | Sildīšanas līkne atspoguļo ēkas siltuma rādītājus. Ieteicamais maisītāja loka iestatījums: 0,2 - 0,6 |
| 102 | Maisītāja vadība 1 | 2 (Grīdas iesl.) | 0=Izslēgts / 1=CA ieslēgš. / 2=Grīdas iesl. / 3=Tikai sūknis | enum | IZSLĒGTS = maisītāja piedziņa un sūknis nedarbojas, CA IESLĒGTS = maisītāja loks piegādā ūdeni uz radiatoru instalāciju, GRĪDA IESLĒGTA = maisītāja loks piegādā ūdeni uz grīdas instalāciju, TIKAI SŪKNIS = piedziņa ir izslēgta, sūknis izslēdzas pēc iestatītās temperatūras sasniegšanas. |
| 103 | Maisītāja vadība 2 | 2 (Grīdas iesl.) | 0=Izslēgts / 1=CA ieslēgš. / 2=Grīdas iesl. / 3=Tikai sūknis | enum | IZSLĒGTS = maisītāja piedziņa un sūknis nedarbojas, CA IESLĒGTS = maisītāja loks piegādā ūdeni uz radiatoru instalāciju, GRĪDA IESLĒGTA = maisītāja loks piegādā ūdeni uz grīdas instalāciju, TIKAI SŪKNIS = piedziņa ir izslēgta, sūknis izslēdzas pēc iestatītās temperatūras sasniegšanas. |
| 104 | Maisītāja vadība 3 | 1 (CA ieslēgš.) | 0=Izslēgts / 1=CA ieslēgš. / 2=Grīdas iesl. / 3=Tikai sūknis | enum | IZSLĒGTS = maisītāja piedziņa un sūknis nedarbojas, CA IESLĒGTS = maisītāja loks piegādā ūdeni uz radiatoru instalāciju, GRĪDA IESLĒGTA = maisītāja loks piegādā ūdeni uz grīdas instalāciju, TIKAI SŪKNIS = piedziņa ir izslēgta, sūknis izslēdzas pēc iestatītās temperatūras sasniegšanas. |
| 105 | Maisītāja vadība 4 | 2 (Grīdas iesl.) | 0=Izslēgts / 1=CA ieslēgš. / 2=Grīdas iesl. / 3=Tikai sūknis | enum | IZSLĒGTS = maisītāja piedziņa un sūknis nedarbojas, CA IESLĒGTS = maisītāja loks piegādā ūdeni uz radiatoru instalāciju, GRĪDA IESLĒGTA = maisītāja loks piegādā ūdeni uz grīdas instalāciju, TIKAI SŪKNIS = piedziņa ir izslēgta, sūknis izslēdzas pēc iestatītās temperatūras sasniegšanas. |
| 106 | Maisītāja vadība 5 | 2 (Grīdas iesl.) | 0=Izslēgts / 1=CA ieslēgš. / 2=Grīdas iesl. / 3=Tikai sūknis | enum | IZSLĒGTS = maisītāja piedziņa un sūknis nedarbojas, CA IESLĒGTS = maisītāja loks piegādā ūdeni uz radiatoru instalāciju, GRĪDA IESLĒGTA = maisītāja loks piegādā ūdeni uz grīdas instalāciju, TIKAI SŪKNIS = piedziņa ir izslēgta, sūknis izslēdzas pēc iestatītās temperatūras sasniegšanas. |
| 107 | Proporc. diapazons | 2 | 0..10 |  | Jo augstāka vērtība, jo ātrāk tiek sasniegta uzstādītā maisītāja temperatūra. Pārāk augsta parametra vērtība rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 108 | Proporc. diapazons | 3 | 0..10 |  | Jo augstāka vērtība, jo ātrāk tiek sasniegta uzstādītā maisītāja temperatūra. Pārāk augsta parametra vērtība rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 109 | Proporc. diapazons | 3 | 0..10 |  | Jo augstāka vērtība, jo ātrāk tiek sasniegta uzstādītā maisītāja temperatūra. Pārāk augsta parametra vērtība rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 110 | Proporc. diapazons | 3 | 0..10 |  | Jo augstāka vērtība, jo ātrāk tiek sasniegta uzstādītā maisītāja temperatūra. Pārāk augsta parametra vērtība rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 111 | Proporc. diapazons | 3 | 0..10 |  | Jo augstāka vērtība, jo ātrāk tiek sasniegta uzstādītā maisītāja temperatūra. Pārāk augsta parametra vērtība rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 112 | Integrācijas laiks | 255 | 30..255 |  | Jo mazāka vērtība, jo ātrāk tiek sasniegta iestatītā maisītāja temperatūra. Pārāk mazs parametra lielums rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 113 | Integrācijas laiks | 160 | 30..255 |  | Jo mazāka vērtība, jo ātrāk tiek sasniegta iestatītā maisītāja temperatūra. Pārāk mazs parametra lielums rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 114 | Integrācijas laiks | 160 | 30..255 |  | Jo mazāka vērtība, jo ātrāk tiek sasniegta iestatītā maisītāja temperatūra. Pārāk mazs parametra lielums rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 115 | Integrācijas laiks | 160 | 30..255 |  | Jo mazāka vērtība, jo ātrāk tiek sasniegta iestatītā maisītāja temperatūra. Pārāk mazs parametra lielums rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 116 | Integrācijas laiks | 160 | 30..255 |  | Jo mazāka vērtība, jo ātrāk tiek sasniegta iestatītā maisītāja temperatūra. Pārāk mazs parametra lielums rada pārmērīgu temperatūras svārstību un izpildmehānisma nevajadzīgu kustību. |
| 117 | Sūkņ izsl no termost | 1 (JĀ) | 0=NĒ / 1=JĀ | enum | Ja parametrs ir iestatīts uz JĀ, maisītāja piedziņa tiek aizvērta un maisītāja sūknis tiek izslēgts pēc telpas termostats kontaktu atvēršanas (sasilušā telpā). |
| 118 | Sūkņ izsl no termost | 0 (NĒ) | 0=NĒ / 1=JĀ | enum | Ja parametrs ir iestatīts uz JĀ, maisītāja piedziņa tiek aizvērta un maisītāja sūknis tiek izslēgts pēc telpas termostats kontaktu atvēršanas (sasilušā telpā). |
| 119 | Sūkņ izsl no termost | 1 (JĀ) | 0=NĒ / 1=JĀ | enum | Ja parametrs ir iestatīts uz JĀ, maisītāja piedziņa tiek aizvērta un maisītāja sūknis tiek izslēgts pēc telpas termostats kontaktu atvēršanas (sasilušā telpā). |
| 120 | Sūkņ izsl no termost | 0 (NĒ) | 0=NĒ / 1=JĀ | enum | Ja parametrs ir iestatīts uz JĀ, maisītāja piedziņa tiek aizvērta un maisītāja sūknis tiek izslēgts pēc telpas termostats kontaktu atvēršanas (sasilušā telpā). |
| 121 | Sūkņ izsl no termost | 0 (NĒ) | 0=NĒ / 1=JĀ | enum | Ja parametrs ir iestatīts uz JĀ, maisītāja piedziņa tiek aizvērta un maisītāja sūknis tiek izslēgts pēc telpas termostats kontaktu atvēršanas (sasilušā telpā). |
| 122 | Vārsta atvērš. laiks | 120 | 10..255 | sec. | Jāievada pilnas vārsta atvēršanās laiks, kas norādīts uz vārsta piedziņas marķējuma plāksnītes, piem. 140s. |
| 123 | Vārsta atvērš. laiks | 140 | 10..255 | sec. | Jāievada pilnas vārsta atvēršanās laiks, kas norādīts uz vārsta piedziņas marķējuma plāksnītes, piem. 140s. |
| 124 | Vārsta atvērš. laiks | 140 | 10..255 | sec. | Jāievada pilnas vārsta atvēršanās laiks, kas norādīts uz vārsta piedziņas marķējuma plāksnītes, piem. 140s. |
| 125 | Vārsta atvērš. laiks | 140 | 10..255 | sec. | Jāievada pilnas vārsta atvēršanās laiks, kas norādīts uz vārsta piedziņas marķējuma plāksnītes, piem. 140s. |
| 126 | Vārsta atvērš. laiks | 140 | 10..255 | sec. | Jāievada pilnas vārsta atvēršanās laiks, kas norādīts uz vārsta piedziņas marķējuma plāksnītes, piem. 140s. |
| 127 | Atpakaļg. aizsardz. 4-CV | 0 (OFF) | 0=OFF / 1=ON | enum | 4-CV ir aizvērts ja atpakaļgaitas t. ir par zemu |
| 128 | Min. atpakaļgaitas temp. | 40 | 0..80 | °C | 4D vārsts atgriežas normālā darba režīmā, kad  atpakaļgaitas temperatūra ir paaugstinājusies. |
| 129 | Atpakaļg histerēze | 5 | 1..10 | °C | 4D vārsts atgriežas normālā darba režīmā, kad  atpakaļgaitas temperatūra ir paaugstinājusies. |
| 130 | Vārsta pievēršana | 10 | 0..80 | % | Vārsta aizvēršana kad atpakaļgaitas temperatūra ir pārāk zema. |
| 131 | Katla vadība pēc ārgaisa temp. | 1 (Ieslēgts) | 0=Izslēgts / 1=Ieslēgts | enum | Katla temperatūra tiek noteikta automātiski pēc apkures līkņu principa |
| 132 | Katla apkures līkne | 0.8 | 0.1..4.0 |  | Apkures līkne atspoguļo ēkas siltuma īpašības. Ieteicamā apkures līkne katlam: 1.8 - 4 |
| 133 | Katla atdziš. temp. | 90 | 85..95 | °C | Virs šīs temperatūras regulators ieslēdz SKŪ sūkni un atver maisītāju lokus, lai atvēsinātu  katlu. Regulators izslēdz SKŪ sūkni ja ūdens temperatūra pārsniedz maksimālo vērtību. Regulator neatver maisītāja loku, kad MAISĪTĀJA VADĪBA = grīda ieslēgta. |
| 134 | Termostata izvēle | 12 (Izslēgts) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu tiešajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signals tiek saņemts no ecoSTER200 pults. |
| 135 | Termostata izvēle | 14 (ecoSTER T1) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu regulējamajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signāls tiek saņemts no ecoSTER200 pults. |
| 136 | Termostata izvēle | 12 (Izslēgts) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu regulējamajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signāls tiek saņemts no ecoSTER200 pults. |
| 137 | Termostata izvēle | 15 (ecoSTER T3) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu regulējamajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signāls tiek saņemts no ecoSTER200 pults. |
| 138 | Termostata izvēle | 12 (Izslēgts) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu regulējamajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signāls tiek saņemts no ecoSTER200 pults. |
| 139 | Termostata izvēle | 12 (Izslēgts) | 0=Izslēgts / 1=Universāls / 2=ecoSTER T1 / 3=ecoSTER T2 / 4=ecoSTER T3 / 5=Izslēgts / 6=Universāls / 7=ecoSTER T2 / 8=ecoSTER T3 / 9=Izslēgts / 10=Universāls / 11=ecoSTER T3 / 12=Izslēgts / 13=Universāls / 14=ecoSTER T1 / 15=ecoSTER T3 / 16=Izslēgts / 17=ecoSTER T1 / 18=ecoSTER T2 / 19=ecoSTER T3 / 20=Izslēgts / 21=ecoSTER T2 / 22=ecoSTER T3 / 23=Izslēgts / 24=ecoSTER T3 / 25=Izslēgts / 26=ecoSTER T1 / 27=ecoSTER T3 / 28=Universāls / 29=Universāls / 30=Universāls / 31=Izslēgts / 32=Izslēgts / 33=Izslēgts | enum | Šis parametrs ļauj izvēlēties telpas termostatu regulējamajam apkures lokam. IZSLĒGTS - izslēdz termostata vadību, UNIVERSĀLAIS - nozīmē parastu termostatu, ecoSTER1,2,3 - termostata signāls tiek saņemts no ecoSTER200 pults. |
| 140 | Cirkulācijas sūkņa pauzes laiks | 30 | 0..255 | min. | Nosaka pārtraukuma laiku starp cirkulācijas sūkņu ieslēgšanās cikliem. Šis parametrs ir pieejams pēc papildus C moduļa pieslēgšanas. |
| 141 | Cirkulācijas sūkņa darba laiks | 70 | 0..255 | sec. | Nosaka darbības laiku pēc cirkulācijas sūkņa pārtraukuma laika. Šis parametrs ir pieejams pēc papildus C moduļa pieslēgšanas. |
| 142 | Cirkulācijas sūkņa ieslēgšanas temp. | 45 | 3..65 | °C |  |
| 143 | Paralēlā līknes nobīde | 0.0 | -20.0..20.0 | °C | Ļauj palielināt vai samazināt temperatūru apkures lokā, ja apkures līkne neprecīzi atbilst ēkas īpašībām. |
| 144 | Līkņu sajaukšanās | 0.0 | -20.0..20.0 | °C | Ļauj koriģēt - palielināt/samazināt temperatūru regulējamajā lokā, kad laikapstākļu vadība ir ieslēgta, gadījumā, ja sildīšanas līkne precīzi neatbilst ēkas rādītājiem. |
| 145 | Līkņu sajaukšanās | 0.0 | -20.0..20.0 | °C | Ļauj koriģēt - palielināt/samazināt temperatūru regulējamajā lokā, kad laikapstākļu vadība ir ieslēgta, gadījumā, ja sildīšanas līkne precīzi neatbilst ēkas rādītājiem. |
| 146 | Līkņu sajaukšanās | 0.0 | -20.0..20.0 | °C | Ļauj koriģēt - palielināt/samazināt temperatūru regulējamajā lokā, kad laikapstākļu vadība ir ieslēgta, gadījumā, ja sildīšanas līkne precīzi neatbilst ēkas rādītājiem. |
| 147 | Līkņu sajaukšanās | 0.0 | -20.0..20.0 | °C | Ļauj koriģēt - palielināt/samazināt temperatūru regulējamajā lokā, kad laikapstākļu vadība ir ieslēgta, gadījumā, ja sildīšanas līkne precīzi neatbilst ēkas rādītājiem. |
| 148 | Līkņu sajaukšanās | 0.0 | -20.0..20.0 | °C | Ļauj koriģēt - palielināt/samazināt temperatūru regulējamajā lokā, kad laikapstākļu vadība ir ieslēgta, gadījumā, ja sildīšanas līkne precīzi neatbilst ēkas rādītājiem. |
| 149 | Telpas temperatūras koeficients | 0 | 0..50 |  | Korekcijas koeficients iestatītajai temperatūrai apkures lokā, pamatojoties uz telpas temperatūras sensoru. Jo augstāka vērtība, jo lielāka temperatūras korekcija apkures lokā. Ir jāpievieno ecoSTER telpas panelis. |
| 150 | Telpas temp. Faktors | 0 | 0..50 |  | Koeficients, kas koriģē apkures loka iestatīto temperatūru, balstoties uz telpas temperatūras sensora rādījumiem. Cik lielāka vērtība, tik lielāka apkures loka temperatūras korekcija. Jāpieslēdz ecoSTER telpas pults. |
| 151 | Telpas temp. Faktors | 0 | 0..50 |  | Koeficients, kas koriģē apkures loka iestatīto temperatūru, balstoties uz telpas temperatūras sensora rādījumiem. Cik lielāka vērtība, tik lielāka apkures loka temperatūras korekcija. Jāpieslēdz ecoSTER telpas pults. |
| 152 | Telpas temp. Faktors | 0 | 0..50 |  | Koeficients, kas koriģē apkures loka iestatīto temperatūru, balstoties uz telpas temperatūras sensora rādījumiem. Cik lielāka vērtība, tik lielāka apkures loka temperatūras korekcija. Jāpieslēdz ecoSTER telpas pults. |
| 153 | Telpas temp. Faktors | 0 | 0..50 |  | Koeficients, kas koriģē apkures loka iestatīto temperatūru, balstoties uz telpas temperatūras sensora rādījumiem. Cik lielāka vērtība, tik lielāka apkures loka temperatūras korekcija. Jāpieslēdz ecoSTER telpas pults. |
| 154 | Telpas temp. Faktors | 0 | 0..50 |  | Koeficients, kas koriģē apkures loka iestatīto temperatūru, balstoties uz telpas temperatūras sensora rādījumiem. Cik lielāka vērtība, tik lielāka apkures loka temperatūras korekcija. Jāpieslēdz ecoSTER telpas pults. |
| 155 | Tīrīšanas gaisa padeve | 100 | 85..100 | % | Tīrīšanas gaisa padeve. |
| 156 | Tīrīšanas vilkmes vent. intensitāte | 100 | 16..100 | % |  |
| 157 | Degļa tīrīšana | 24 | 1..250 | h. | Tas ir nepārtrauktas degļa darbības laiks, pēc kura notiek automātiskā izdzēšana, tīrīšana un atkārtota degļa aizdedzināšana. |
| 158 | Ventilatora jauda | 96 | 85..100 | % | Ventilatora jauda izdzēšanas laikā. |
| 159 | Vilkmes jauda pie izdzišanas | 80 | 16..100 | % | Vilkmes jauda izdzišanas laikā |
| 160 | Akumulācij.vadība | 1 (JĀ) | 0=NĒ / 1=JĀ | enum | Ļauj ieslēgt vai izslēgt darbību ar buferi. |
| 161 | Akumul. uzsildes temp. | 46 | 40..85 | °C | Šis parametrs definē maksimālo bufera temperatūru, zem kuras sāksies bufera uzpilde. Bufera uzpilde beidzas kad minimālā bufera temperatūra sasniedz parametrā BUFERA UZPILDES BEIGU TEMPERATŪRA iestatīto vērtību. |
| 162 | Akumul. augšējā temp. | 72 | 40..85 | °C | Šis parametrs definē minimālo bufera temperatūru, virs kuras beigsies bufera uzpilde. Bufera uzpilde sākas atkal kad maksimālā bufera temperatūra nokritusi zem temperatūru BUFERA UZPILDES SĀKUMA TEMPERATŪRA. |
| 163 | Trauksmes sign līmen | 5 | 5..99 | % | Sliekšņa līmenis degvielai ar kuru pamudināja: zems degvielas! |
| 164 | Degvielas trūkuma noteikšanas laiks | 1 | 0..5 | min. | Laiks tiek atskaitīts pēc tam, kad liesmas gaišums ir nokritis zem parametra "Liesmas detekcija" vērtības. Pēc laika atskaitīšanas regulators mēģina aizdedzināt degli, un pēc 3 nesekmīgiem mēģinājumiem, attēlo brīdinājumu "Nesekmīgs aizdedzināšanas mēģinājums". |
| 165 | Maisītāja histerēze 1 | 2.0 | 0.0..4.0 | °C | Parametra iestatījums, kas noteic temperatūras nejūtīguma vērtību (sastinguma punkts) maisītāja vadības sistēmai. Regulēšana notiek tad, kad izmērītā loka temperatūra ir augstāka vai zemāka par iestatīto temperatūru par vērtību MAISĪTĀJA NEJŪTĪGUMS. |
| 166 | Maisītāja histerēze 2 | 2.0 | 0.0..4.0 | °C | Parametra iestatījums, kas noteic temperatūras nejūtīguma vērtību (sastinguma punkts) maisītāja vadības sistēmai. Regulēšana notiek tad, kad izmērītā loka temperatūra ir augstāka vai zemāka par iestatīto temperatūru par vērtību MAISĪTĀJA NEJŪTĪGUMS. |
| 167 | Maisītāja histerēze 3 | 2.0 | 0.0..4.0 | °C | Parametra iestatījums, kas noteic temperatūras nejūtīguma vērtību (sastinguma punkts) maisītāja vadības sistēmai. Regulēšana notiek tad, kad izmērītā loka temperatūra ir augstāka vai zemāka par iestatīto temperatūru par vērtību MAISĪTĀJA NEJŪTĪGUMS. |
| 168 | Maisītāja histerēze 4 | 2.0 | 0.0..4.0 | °C | Parametra iestatījums, kas noteic temperatūras nejūtīguma vērtību (sastinguma punkts) maisītāja vadības sistēmai. Regulēšana notiek tad, kad izmērītā loka temperatūra ir augstāka vai zemāka par iestatīto temperatūru par vērtību MAISĪTĀJA NEJŪTĪGUMS. |
| 169 | Maisītāja histerēze 5 | 2.0 | 0.0..4.0 | °C | Parametra iestatījums, kas noteic temperatūras nejūtīguma vērtību (sastinguma punkts) maisītāja vadības sistēmai. Regulēšana notiek tad, kad izmērītā loka temperatūra ir augstāka vai zemāka par iestatīto temperatūru par vērtību MAISĪTĀJA NEJŪTĪGUMS. |
| 170 | Padevēja laiks 2 | 0 | 0..100 | min. | Papildus padevēja (ārējā) darbības laiks. |
| 171 | Rezerves katls | 0 | 25..60 | °C | Noteic brikešu katla temperatūru, zem kuras tiks ieslēgts rezerves katls (piem. gāzes katls). Ja iestatījums ir "0" - rezerves katla vadība ir izslēgta un izeja 39-40 ir iestatīta uz signalizāciju. |
| 172 | Degvielas izvēle | 1 (XS 20kW) | 0=XS 16kW / 1=XS 20kW / 2=XS 26kW / 3=XS 36kW / 4=XS 50kW / 5=XS 70kW / 6=XS 100kW / 7=XS 150kW / 8=XS 200kW / 9=XS 250kW | enum | Šis parametrs ļauj mainīt degvielu. Izvēlētajam degvielas veidam tiks ielādēti jauni iestatījumi izvēlnē: JAUDAS MODULĀCIJA. |
| 173 | Maksimālā degļa temperatūra | 60 | 40..90 | °C | Noteic maksimālo degļa temperatūru, kuru sasniedzot parādīsies brīdinājums par maksimālo degļa temperatūru pārsniegšanu. |
| 174 | Minimālais padevēja 2. darbības laiks | 6 | 0..250 | sec. | Deglī uzstādīta padevēja 2. minimālais darbības laiks. |
| 175 | Tīrīšanas laiks Aizdedzināšana | 10 | 10..250 | sec. | Tīrīšanas laiks Aizdedzināšana. |
| 176 | Tīrīšanas laiks Izdzēšana | 2 | 1..30 | min. | Tīrīšanas laiks Izdzēšana. |
| 177 | Ventilatora darbības laiks | 40 | 1..100 | sec. | Ventilatora darbības laiks sadedzinot degvielu IZDZĒŠANAS REŽĪMĀ |
| 178 | Ventilatora darbības pārtraukums | 15 | 10..250 | sec. | Pārtraukums starp ventilatora darbības cikliem sadedzinot degvielu IZDZĒŠANAS REŽĪMĀ. |
| 179 | Ventilatora ieslēgšanās | 10 | 1..100 | % | Liesmas spilgtums, pie kura ventilators ieslēdzas, kad notiek degvielas sadedzināšana IZDZĒŠANAS REŽĪMĀ. |
| 180 | Ventilatora apstāšanās | 5 | 1..100 | % | Liesmas spilgtums, pie kura ventilators apstājas, kad notiek degvielas sadedzināšana IZDZĒŠANAS REŽĪMĀ. |
| 181 | Režģa režīms | 1 (ON) | 0=OFF / 1=ON | enum |  |
| 182 | Siltuma avots | 1 (Režģis - Koksne) | 0=Deglis - Briketes / 1=Režģis - Koksne / 2=Rezerves katls | enum | Siltuma avota izvēle. |
| 183 | Automātiska pāreja uz granulām | 1 (ON) | 0=OFF / 1=ON | enum |  |
| 184 | Uzstādītā Katla temperatūra | 80 | 40..85 | °C | Iestatītā katla temperatūra kad laikapstākļu vadība ir izslēgta. Var tikt automātiski mainīta ar regulatoru, piem. kad iestatītā katla temperatūra ir zemāka par iestatīto SKŪ temperatūru. |
| 185 | Degvielas trūkuma noteikšanas laiks | 15 | 0..250 | min. | Laiks tiek atskaitīts pēc tam, kad liesmas gaišums ir nokritis zem parametra "Liesmas detekcija" vērtības. Pēc laika atskaitīšanas regulators mēģina aizdedzināt degli, un pēc 3 nesekmīgiem mēģinājumiem, attēlo brīdinājumu "Nesekmīgs aizdedzināšanas mēģinājums". |
| 186 | Vent. Jauda | 86 | 85..100 | % | Ventilatora jauda režīmā Režģis |
| 187 | Vilkmes jauda - malkas režīms | 30 | 16..100 | % | Vilkmes jauda malkas režīmā |
| 188 | Katla histerēze | 4 | 1..30 | °C | Ja katla temperatūra ir pazeminājusies zem iestatītās katla temperatūras par katla histerēzes vērtību, notiek automātiskā degļa aizdedzināšana. |
| 189 | Ventilatora darbība - gaidīšana | 20 | 0..100 | sec. | Ventilatora darbības laiks kad notiek GAIDĪŠANA darbojoties režīmā REŽĢIS. Vērtībai nav jābūt pārāk lielai, lai neizraisītu ūdens pārkaršanu katlā. |
| 190 | Ventilatora darbības pārtraukums - gaidīšana | 8 | 1..250 | min. | Ventilatora darbības pārtraukuma laiks kad notiek GAIDĪŠANA darbojoties režīmā REŽĢIS. Vērtībai nav jābūt pārāk mazai, lai neizraisītu ūdens pārkaršanu katlā. |
| 191 | Lambda režīms | 1 (Ieslēgt) | 0=Izslēgt / 1=Ieslēgt | enum | Ieslēdz vai izslēdz Lambda zondes darbību. |
| 192 | Katla termostata funkcija | 2 (Izslēdziet degli un sūkni) | 0=Izslēdziet degli / 1=Izslēdziet sūkni / 2=Izslēdziet degli un sūkni | enum |  |
| 193 | Režīms termostats | 0 (OFF) | 0=OFF / 1=ON | enum |  |
| 194 | Vilkmes ventilators | 1 (ON) | 0=OFF / 1=ON | enum |  |
| 195 | Parādīt izvērsto | 1 (JĀ) | 0=NĒ / 1=JĀ | enum | Ja iestatīsit YES, atsevišķos izvēlnes elementos tiks parādīti paslēpti papildu parametri. |
| 196 | Sensora korekcija | 0.0 | -5.0..5.0 | °C | Āra temp. sensora korekcija |
| 197 | Ražotājs | 1 (KIPI) | 0=PLUM / 1=KIPI / 2=BLAZE HARMONY / 3=GALMET / 4=KOTŁOSPAW / 5=SILDI | enum |  |
| 198 | H izejas konfigurācija | 0 (OFF) | 0=OFF / 1=Cirkulācijas sūknis / 2=Trauksme / 3=Rezerves katls / 4=Cirkulācijas sūknis / 5=Padevējs 2 | enum |  |
| 199 | H izejas konfigurācija (mod) | 0 (OFF) | 0=OFF / 1=Cirkulācijas sūknis / 2=Trauksme / 3=Rezerves katls / 4=Cirkulācijas sūknis / 5=Padevējs 2 | enum | H izejas konfigurācija |
| 200 | Siltummaiņa tīrīšanas intervāls | 100.0 | 10.0..2500.0 | kg |  |
| 201 | Siltummaiņa tīrīšanas laiks | 30 | 1..240 | sec. |  |
| 202 | Tīrīšanas beigu laiks | 23 | 0..23 |  | Siltummaiņa tīrīšanas beigu laiks |
| 203 | Tīrīšanas sākuma laiks | 0 | 0..23 |  | Siltummaiņa tīrīšanas sākuma laiks |
| 204 | Siltummaiņa tīrīšanas trauksme | 400.0 | 10.0..2500.0 | kg | Degvielas daudzumu aktivizē trauksme, lai notīrītu tvertni. |
| 205 | Cirkulācijas sūknis sākt | 35 | 0..60 | °C | Cirkulācijas sūkņa aktivācijas temperatūra |

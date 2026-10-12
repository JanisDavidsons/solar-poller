' Runs the watchdog without flashing a console window (scheduled task calls this).
Set sh = CreateObject("WScript.Shell")
sh.Run "powershell -NoProfile -ExecutionPolicy Bypass -File ""C:\Users\janis\home-energy-monitor\scripts\telegraf-watchdog.ps1""", 0, False

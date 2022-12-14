pyinstaller.exe manager_GUI.py --noconsole --noconfirm

if exist .\waypoint_manager PowerShell -command " rm -r .\waypoint_manager "
PowerShell -command " mv .\dist\manager_GUI .\waypoint_manager "
PowerShell -command " rm -r .\build "
PowerShell -command " rm -r .\dist "
PowerShell -command " rm .\manager_GUI.spec "
PowerShell -command " cp -r .\icons .\waypoint_manager\ "







python -m PyInstaller manager_GUI.py --noconsole --noconfirm

if [ -d ./waypoint_manager ]; then
    rm -r ./waypoint_manager
fi

mv ./dist/manager_GUI ./waypoint_manager
rm -r ./build
rm -r ./dist
rm ./manager_GUI.spec
cp -r ./icons ./waypoint_manager/

# If the ModuleNotFoundError occur, copy the module from your environment

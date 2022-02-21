* en cas de plantage d'installation de package (defectueux, garder en l'etat...)

sudo dpkg --force-all --purge "the package"

Problem vscode : cursor moves left one place !!

    Change keyboard to : "keyboard.dispatch": "keyCode"
    
If sudo gedit fails :

    type on terminal : xhost +local:
    
To remove power saving options : 

	sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
	
		Created symlink /etc/systemd/system/sleep.target → /dev/null.
		Created symlink /etc/systemd/system/suspend.target → /dev/null.
		Created symlink /etc/systemd/system/hibernate.target → /dev/null.
		Created symlink /etc/systemd/system/hybrid-sleep.target → /dev/null.
		
	reboot and verify disables :
	
	sudo systemctl status sleep.target suspend.target hibernate.target hybrid-sleep.target
	
		

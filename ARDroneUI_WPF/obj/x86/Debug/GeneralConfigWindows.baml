   M S B A M L   `   `   `  ����O  Kmscorlib, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e0891&clr-namespace:System;assembly=mscorlibSystem  5   	   E AARDrone.UI, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null'clr-namespace:ARDrone.UI
ARDrone.UI 5   	   J FARDrone.Control, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullT;clr-namespace:ARDrone.Control.Data;assembly=ARDrone.ControlARDrone.Control.Data 5   	   $   ARDrone.UI.GeneralConfigWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35^ ZWindowsFormsIntegration, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   6sys&clr-namespace:System;assembly=mscorlib     5   	   *localclr-namespace:ARDrone.UI     5   	   Mdrone;clr-namespace:ARDrone.Control.Data;assembly=ARDrone.Control     5   	     � Title$  Config Settings��5   	    � 
ResizeMode$ NoResize=�6!    � SizeToContent$ WidthAndHeight=�67    � Icon$+ %/ARDrone.UI;component/Images/Icon.png��6V    � 	Resources 5       �� 
TargetType �� Property �� Value �� ElementName	 �� Path  ARDrone.UI.EnumDisplayer
   Type3  -ARDrone.Control.Data.SupportedFirmwareVersion   firmwareVersions��%  '��       &	  �    �� 5   
   " ��6   .�5      ��  �@=�ErrorTemplate 5      �� 5      .R�5      �� .+�5      �� 
���  ��$��1q�67   $
��Left=�6�   ��6K   �� $ Adorner��$	 AdornedElement.Width�._�5   "   �� $��Adorner��6<   5      �� 5      $��Right=�6(   
���  ��$��8(�6�   ��6?   �� $ Adorner��$8	 2AdornedElement.(Validation.Errors)[0].ErrorContent�6�   5      5      5       �� 5!       � ��$ 0,0,0,12q�6'   5"        5$   
   "
  6   6M   5%      .��5'        ARDrone.UI.ErrorProvider  ._�5(   
   �  � ColumnDefinitions 5)      �� 5*      $��* �6#    � RowDefinitions 5,      �� 5-      $
��Auto �6    �� 5.      $
��Auto �6    �� 5/      $
��Auto �6    .+�51      �� $��0��6   $��Drone settings��6$   $��3q�6<   .��52      �  53      �� 54      $
��Auto �6+   �� 55      $	��200 �6+    57      �� 58      $
��Auto �6(   �� 59      $
��Auto �6(   �� 5:      $
��Auto �6(   �� 5;      $
��Auto �6(   �� 5<      $
��Auto �6(   �� 5=      $
��Auto �6(   �� 5>      $
��Auto �6(   �� 5?      $
��Auto �6(   �� 5@      $
��Auto �6(   �� 5A      $
��Auto �6(   .+�5C      �� $"��Drone network SSID contains:��6   $��0��6C   $��0��6P   �� 5D      -   5        $8��2textBoxDroneSettingDroneNetworkIdentifierStartText��5D      $��0��6Y   $��1��6f   .��6v   .��5E      �� $	 DroneNetworkSSID�6"    �� UpdateSourceTrigger$ PropertyChanged=�6:   5F      �� 5H      $��Drone IP address��6   $��1��67   $��0��6D   �� 5I      -   5        $&�� textBoxDroneSettingsOwnIpAddress��5I      $��1��6F   $��1��6S   .��6c   .��5J      �� $	 DroneIpAddress�6"   $ PropertyChanged=�68   5K      �� 5M      $��2��6!   $��0��6.   .+�5N      �� $��Own IP address��6    �� 5O      $,��&(used only if no IP address was found)��6    $��8(�6Q   5P      �� 5R      -   5        $(��"textBoxDroneSettingsDroneIpAddress��5R      $��2��6H   $��1��6U   $	��Top=�6e   .��6}   .��5S      �� $	 OwnIpAddress�6"   $ PropertyChanged=�66   5T      �� 5V      $��Command data port��6   $��3��68   $��0��6E   �� 5W      -   5        $)��#textBoxDroneSettingsCommandDataPort��5W      $��3��6I   $��1��6V   $	��100��6f   $
��Left=�6r   .��6�   .��5X      �� $	 CommandPortText�6"   $ PropertyChanged=�69   5Y      �� 5[      $��Navigation data port��6   $��4��6;   $��0��6H   �� 5\      -   5        $,��&textBoxDroneSettingsNavigationDataPort��5\      $��4��6L   $��1��6Y   $	��100��6i   $
��Left=�6u   .��6�   .��5]      �� $	 NavigationPortText�6"   $ PropertyChanged=�6<   5^      �� 5`      $��Video data port��6   $��5��66   $��0��6C   �� 5a      -   5        $'��!textBoxDroneSettingsVideoDataPort��5a      $��5��6G   $��1��6T   $	��100��6d   $
��Left=�6p   .��6�   .��5b      �� $	 VideoPortText�6"   $ PropertyChanged=�67   5c      �� 5e      $��Control info port��6   $��6��68   $��0��6E   �� 5f      -   5        $)��#textBoxDroneSettingsControlInfoPort��5f      $��6��6I   $��1��6V   $	��100��6f   $
��Left=�6r   .��6�   .��5g      �� $	 ControlPortText�6"   $ PropertyChanged=�69   5h      �� 5j      -   5        $#��Use specific firmware version��5j      $��7��6G   $��5q�6T   $��2��6�    d� 	IsChecked 6_   �� *UseSpecificFirmwareVersion+6�   �� 5l      $��Drone firmware version��6   $��8��6=   $��0��6J   �� 5m      -	   5        $*��$comboBoxDroneSettingsFirmwareVersion��5m      $��8��6K   $��1��6X   ��6h   ��  �� Source# [  $	 DisplayNames� �� SelectedValue 6�   �� $	 FirmwareVersion� �� 	Converter# [  5o      5p      �� 5q      $��1��6   $��HUD settings��6$   $��3q�6:   .��5r      �  5s      �� 5t      $��* �6+    5v      �� 5w      $
��Auto �6(   �� 5x      $
��Auto �6(   �� 5y      $
��Auto �6(   �� 5z      $
��Auto �6(   �� 5{      $
��Auto �6(   �� 5|      $
��Auto �6(   �� 5}      $
��Auto �6(   .+�5      �� -
   5        $ ��checkBoxHudSettingsShowHud��5   ?   $��Show HUD��6   $��0��62   $��5q�6a    6�   �� *	ShowHud+6�   �� 5�      -   5        $#��checkBoxHudSettingsShowTarget��5�   B   $��Show target��6   $��1��65   $��25, 2, 2, 2q�6g    6|   �� *ShowHudTarget+�� 5�      -   5        $%��checkBoxHudSettingsShowBaseLine��5�   D   $��Show baseline��6   $��2��67   $��25, 2, 2, 2q�6k    6�   �� *ShowHudBaseLine+�� 5�      -   5        $$��checkBoxHudSettingsShowHeading��5�   C   $��Show heading��6   $��3��66   $��25, 2, 2, 2q�6i    6~   �� *ShowHudHeading+�� 5�      -   5        $%��checkBoxHudSettingsShowAltitude��5�   D   $��Show altitude��6   $��4��67   $��25, 2, 2, 2q�6k    6�   �� *ShowHudAltitude+�� 5�      -   5        $"��checkBoxHudSettingsShowSpeed��5�   A   $��
Show speed��6   $��5��64   $��25, 2, 2, 2q�6e    6z   �� *ShowHudSpeed+�� 5�      -   5        $)��#checkBoxHudSettingsShowBatteryLevel��5�   I   $��Show battery level��6   $��6��6<   $��25, 2, 2, 2q�6t    6�   �� *ShowHudBattery+5�      5�      �� 5�      $��2��6    �� Orientation$ 
Horizontal=�6&   $��Right=�6?   .+�5�      �� -   5        $��buttonCancel��5�      $��Cancel��6-   $��5q�6>   $��Right=�6I   6e   �� 5�      -   5        $��buttonSubmit��5�      $��Submit��6-   $��5q�6>   $��Right=�6I   6e   5�      5�      5�      5�      
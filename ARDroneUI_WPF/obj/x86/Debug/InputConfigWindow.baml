   M S B A M L   `   `   `  ����  
ARDrone.UI"    ARDrone.UI.InputConfigDialog   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35^ ZWindowsFormsIntegration, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  Configure Input Devices��5   	   $	��545��6)   $	��545��65    � ShowInTaskbar .  � SizeToContent$ Height=�6�    � Topmost .   � 
ResizeMode$ NoResize=�6�    � Icon$+ %/ARDrone.UI;component/Images/Icon.png��6�   .��5      � -   5        $��gridMain��5      ���	#FFF0F0F0 � 	Resources 5   
      styleContentLabel �� 
TargetType �� Property	 �� Value  styleContentTextBox  styleContentComboBox��%�  &	        &	 x     &	     �� 5      " ��6.   .�5      ��  � ��$	 2,4,0,0q�6,   �� 5       � ��$	 	10,3,10,3q�6+   5      �� 5      " ��60   .�5      ��  � ��$	 2q�6,   �� 5       � ��$	 	10,3,25,3q�6+   �� 5       � ��$	 Stretch=�68   5      �� 5      " ��61   .�5      ��  � ��$	 2q�6,   �� 5       � ��$	 	10,3,25,3q�6+   �� 5       � ��$	 Stretch=�68   5      5       
 � RowDefinitions
 5"   
   �� 5#      $
��Auto �6   6,   �� 5$      $
��Auto �6   6,   �� 5%      $
��Auto �6   6,    � ColumnDefinitions 5(   
   �� 5)      $��* �6   6+   �� 5*      $��* �6   6+   .+�5-   
   �� $��0��6   $��0��6"    �� Orientation$ 
Horizontal=�62   $
��Left=�6K   .+�5.      �� $��Device:��6   $��4q�6&   $��2q�61   �� 5/      -   5        $��comboBoxDevices��5/      $��4q�6.   $��2q�69   $	��175��6E   .@�50      ��  �� 
IsSelected . {�.  .��6L   -- No device selected --6f   51      52      �� 54   
   -   5        $��labelDevicePresentInfo��54      $�� ��6.   $��0��69   $��1��6F   $��Right=�6V   $��1q�6r   
���  ��6}   � 56   
   -   5        $��gridCommands��56      $��1��6#   $��2��60   58      �� 59   
   $��2��6   $��0��6"   $
��Left=�62   $ 
Horizontal=�6M   $��5q�6f   .+�5:      �� -   5        $��buttonReset��5:      $��Reset��6(   $��3q�68   $��2q�6C   $
��Left=�6O   6e   5;      �� 5<   
   $��2��6   $��1��6"   $��Right=�62   $ 
Horizontal=�6N   $��5q�6g   .+�5=      �� -   5        $��buttonCancel��5=      $��Cancel��6)   $��3q�6:   $��2q�6E   $
��Left=�6Q   6g   �� 5>      -   5        $��buttonSubmit��5>      $��Submit��6)   $��0,3,3,3q�6:   $��2q�6K   $
��Left=�6W   6m   5?      5@      5A      
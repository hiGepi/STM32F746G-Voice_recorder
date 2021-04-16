# Carte SD

Dans cette partie, nous verrons l'ajout de la bibliothèque pour l'utilisation de la carte SD ainsi que son utilisation.
La communication avec la carte SD se fera à l'aide de l'interface SDMMC ainsi que le DMA.

## Documentation

### SDMMC

Rapide, flexible, plus complexe et plus d'E/S

<center>![Diagramme SDMMC](img/sd/sdmmc_diagram.PNG)</center>
*Diagramme SDMMC*

[Lien doc SDMMC](https://www.st.com/content/ccc/resource/training/technical/product_training/group0/a7/61/d8/cf/a0/c8/4d/08/STM32F7_Peripheral_SDMMC/files/STM32F7_Peripheral_SDMMC.pdf/_jcr_content/translations/en.STM32F7_Peripheral_SDMMC.pdf)

### FatFs
[Lien doc FatFs](https://www.st.com/resource/en/user_manual/dm00105259-developing-applications-on-stm32cube-with-fatfs-stmicroelectronics.pdf)

## Ajout la bibliothèque SD

[Vidéo de démonstration](https://www.youtube.com/watch?v=I9KDN1o6924&t=474s&ab_channel=STMicroelectronics)


On se rend sur l'interface de configuration graphique de STM32CubeMX, puis on active le périphérique SDMMC1 dans Connectivity.

![Configuration SDMMC1](img/sd/config_sdmmc.png)

On ajoute la communication par DMA en gardant les paramètres par défaut.

<center>![Configuration DMA RX et TX](img/sd/config_dma.png)</center>

Pour enfin ajouter le module FatFs

![Configuration FatFs](img/sd/config_fatfs.png)

en activant le template DMA.

<center>![Configuration FatFs](img/sd/config_fatfs_2.png)</center>

Enfin, on n'oubliera pas de changer la configuration du connecteur dans Project Manager.

<center>![Configuration Linker](img/sd/config_linker.png)</center>

## Fonctions principales

![ ](img/sd/)
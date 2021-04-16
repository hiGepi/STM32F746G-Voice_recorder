# Audio

Dans cette partie, nous verrons l'ajout de la bibliothèque pour l'utilisation du codec audio WM8994 ainsi que son utilisation.
La communication avec le codec se fera à l'aide de l'interface SAI.

## Documentation 

### Codec WM8994

[Lien doc WM8994](https://www.mouser.com/datasheet/2/76/WM8994_ProductBrief_2-217999.pdf)

### SAI

![Diagramme SAI](img/audio/sai_diagram.png)
*Diagramme SAI*

### DMA
[Lien doc DMA](https://www.st.com/content/ccc/resource/training/technical/product_training/group0/ce/9e/fe/44/e5/62/45/34/STM32F7_System_DMA/files/STM32F7_System_DMA.pdf/jcr:content/translations/en.STM32F7_System_DMA.pdf)
## Ajout de la bibliothèque

On commence par activer l'interface SAI dans Multimedia avec une liaison Maitre et une laison Esclave synchrone.

![Diagramme SAI](img/audio/config_sai.png)

Les requêtes se font 
![Diagramme SAI](img/audio/config_sai_2.png)

## Fonctions principales
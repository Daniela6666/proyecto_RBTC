INFORME DE ROBOTICA MOVIL — PROYECTO RBTC

Abrir en Overleaf:
1. New Project > Upload Project y seleccionar el ZIP completo.
2. Seleccionar main.tex como documento principal.
3. Compilador: pdfLaTeX. Recompilar (dos pasadas si se usa localmente).

El PDF compilado se incluye como Informe_Robotica_Movil.pdf.
No se necesita ejecutar Python para compilar el informe: las figuras ya estan incluidas.

Contenido:
- main.tex: informe editable, practicas P1-P5 y proyecto final.
- figures/: captura propia de Gazebo, graficas y evidencia TF del repositorio.
- data/: mediciones JSON/CSV, mapas y registros de validacion.
- configuracion/: copia de los parametros locales evaluados.
- fuentes/: informes originales del repositorio, para trazabilidad historica.
- make_figures.py: regeneracion opcional de graficas con matplotlib, numpy, Pillow y PyYAML.

Las cifras de P1/P2 se atribuyen al informe de Persona A del robot actual.
Las cifras de B/C corresponden a la ultima tanda aislada. Se documentan resultados
observados y limites; no se inventaron mediciones ni se incluyeron resultados del
modelo anterior. El esquema de waypoints no es una trayectoria medida.

La configuracion refleja el estado local evaluado al redactar el informe; incluye
ajustes posteriores a la primera integracion en GitHub. Los documentos originales
de B/C contienen notas historicas superadas por los datos actuales. El informe
principal integra y explica estos cambios.

Fecha de las mediciones: 22 de noviembre de 2026.

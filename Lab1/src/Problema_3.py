lista_nombres = ["Pepito", "Pedrito", "Juanito", "Carlitos", "Fulanito","Menguanito"]

lista_sueldo_hora= [50,76,10,14,67,55]

lista_horas_trabajadas = [4,8,9,5,1,7]

for i in range(6):
    
    print(str(lista_nombres[i]) + " will be paid " + str(lista_horas_trabajadas[i]*lista_sueldo_hora[i])+ "$")


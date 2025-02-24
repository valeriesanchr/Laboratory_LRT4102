#le decimos al usuario que nos dé los datos necesarios
n_horas = int(input("¿Cuál es el número de horas trabajadas? "))
n_paga = int(input("¿Cuánto te pagan por hora? "))

#calculamos el pago total
pago_total = n_paga*n_horas

#imprimimos el resultado
print(f"La paga que te corresponde es de ${pago_total}")
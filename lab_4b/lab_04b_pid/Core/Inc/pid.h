/*
* pid.h
*
* Created on: 09.03.2018
* Author: Wojciech Domski
*/

#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include <limits.h>

/**
 * @brief Struktura regulatora PID
 *
 * Ta struktura zawiera wszystkie parametry i zmienne stanu
 * potrzebne do działania regulatora PID z arytmetyką stałoprzecinkową.
 */
typedef struct
{
    int32_t p; /**< Wzmocnienie proporcjonalne (przeskalowane przez potęgę 2) */
    int32_t i; /**< Wzmocnienie całkujące (przeskalowane przez potęgę 2) */
    int32_t d; /**< Wzmocnienie różniczkujące (przeskalowane przez potęgę 2) */

    int32_t p_val; /**< Aktualna wartość składowej proporcjonalnej */
    int32_t i_val; /**< Aktualna wartość składowej całkującej */
    int32_t d_val; /**< Aktualna wartość składowej różniczkującej */

    int32_t p_max; /**< Maksymalne ograniczenie dla składowej proporcjonalnej */
    int32_t i_max; /**< Maksymalne ograniczenie dla składowej całkującej (anti-windup) */
    int32_t d_max; /**< Maksymalne ograniczenie dla składowej różniczkującej */

    int32_t p_min; /**< Minimalne ograniczenie dla składowej proporcjonalnej */
    int32_t i_min; /**< Minimalne ograniczenie dla składowej całkującej (anti-windup) */
    int32_t d_min; /**< Minimalne ograniczenie dla składowej różniczkującej */

    uint8_t f;      /**< Liczba bitów ułamkowych dla arytmetyki stałoprzecinkowej */
    uint32_t power; /**< Współczynnik skalowania (2^f) dla konwersji stałoprzecinkowej */

    int32_t dv; /**< Wartość zadana (setpoint) */
    int32_t mv; /**< Wartość mierzona (zmienna procesowa) */

    int32_t e_last; /**< Poprzednia wartość błędu dla obliczeń różniczkujących */
    int32_t sum;    /**< Skumulowana suma błędów dla obliczeń całkujących */

    int32_t total_max; /**< Maksymalne ograniczenie dla całkowitego wyjścia PID */
    int32_t total_min; /**< Minimalne ograniczenie dla całkowitego wyjścia PID */

    int32_t control; /**< Końcowa wartość wyjściowa sterowania */

    int32_t dt_ms; /**< Czas próbkowania w milisekundach */
} cpid_t;

/**
 * @brief Inicjalizacja regulatora PID
 *
 * Inicjalizuje strukturę regulatora PID z podanymi parametrami.
 * Konwertuje wzmocnienia zmiennoprzecinkowe na reprezentację stałoprzecinkową.
 *
 * @param pid Wskaźnik do struktury regulatora PID
 * @param p Wzmocnienie proporcjonalne (zmiennoprzecinkowe)
 * @param i Wzmocnienie całkujące (zmiennoprzecinkowe)
 * @param d Wzmocnienie różniczkujące (zmiennoprzecinkowe)
 * @param f Liczba bitów ułamkowych dla arytmetyki stałoprzecinkowej
 * @param dt_ms Czas próbkowania w milisekundach
 */
void pid_init(cpid_t *pid, float p, float i, float d, uint8_t f, int32_t dt_ms);

/**
 * @brief Oblicza wyjście sterowania PID
 *
 * Wykonuje jedną iterację obliczeń PID używając aktualnej
 * wartości mierzonej i wartości zadanej (setpoint).
 *
 * @param pid Wskaźnik do struktury regulatora PID
 * @param mv Wartość mierzona (zmienna procesowa)
 * @param dv Wartość zadana (setpoint)
 * @return Wartość wyjściowa sterowania
 */
int32_t pid_calc(cpid_t *pid, int32_t mv, int32_t dv);

/**
 * @brief Skaluje wartość zmiennoprzecinkową do stałoprzecinkowej
 *
 * Konwertuje wartość zmiennoprzecinkową na reprezentację stałoprzecinkową
 * używaną przez regulator PID.
 *
 * @param pid Wskaźnik do struktury regulatora PID (dla współczynnika skalowania)
 * @param v Wartość zmiennoprzecinkowa do przeskalowania
 * @return Przeskalowana wartość stałoprzecinkowa
 */
int32_t pid_scale(cpid_t *pid, float v);

#endif /* PID_H_ */
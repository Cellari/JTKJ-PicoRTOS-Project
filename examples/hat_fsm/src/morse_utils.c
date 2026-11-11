#include "morse_utils.h"
#include <string.h>

// Morse dictionary: each letter or number has a Morse code.
typedef struct {
    char character;
    const char *code;
} MorseEntry;

// Morse code dictionary from:
// https://www.programmersranch.com/2013/05/c-basics-morse-code-converter-using.html
// Credit to the original author for the dictionary used below.
static const MorseEntry morseDictionary[] = {
    {'A', ".-"},
    {'B', "-..."},
    {'C', "-.-."},
    {'D', "-.."},
    {'E', "."},
    {'F', "..-."},
    {'G', "--."},
    {'H', "...."},
    {'I', ".."},
    {'J', ".---"},
    {'K', "-.-"},
    {'L', ".-.."},
    {'M', "--"},
    {'N', "-."},
    {'O', "---"},
    {'P', ".--."},
    {'Q', "--.-"},
    {'R', ".-."},
    {'S', "..."},
    {'T', "-"},
    {'U', "..-"},
    {'V', "...-"},
    {'W', ".--"},
    {'X', "-..-"},
    {'Y', "-.--"},
    {'Z', "--.."},

    {'0', "-----"},
    {'1', ".----"},
    {'2', "..---"},
    {'3', "...--"},
    {'4', "....-"},
    {'5', "....."},
    {'6', "-...."},
    {'7', "--..."},
    {'8', "---.."},
    {'9', "----."}
};

#define MORSE_DICTIONARY_SIZE \
    (sizeof(morseDictionary) / sizeof(morseDictionary[0]))

int morse_to_text(const char *morse, char *output, size_t output_size)
{
    size_t output_index = 0;
    size_t i = 0;

    if (morse == NULL || output == NULL || output_size == 0) {
        return 0;
    }

    output[0] = '\0';

    while (morse[i] != '\0') {

        // Check whether spaces separate letters or words.
        if (morse[i] == ' ') {
            int space_count = 0;

            while (morse[i] == ' ') {
                space_count++;
                i++;
            }

    // Two spaces mean a new word.
    if (space_count >= 2 && output_index > 0) {
        if (output_index + 1 >= output_size) {
            output[0] = '\0';
            return 0;
        }

        output[output_index++] = ' ';
    }
    continue;
}
        // Read one complete Morse symbol.
        char symbol[8];
        size_t symbol_index = 0;

        while (morse[i] != '\0' && morse[i] != ' ' && morse[i] != '/') {

            if (symbol_index >= sizeof(symbol) - 1) {
                output[0] = '\0';
                return 0;
            }

            if (morse[i] != '.' && morse[i] != '-') {
                output[0] = '\0';
                return 0;
            }

            symbol[symbol_index++] = morse[i];
            i++;
        }

        symbol[symbol_index] = '\0';

        // Find the matching letter in the dictionary.
        int found = 0;

        for (size_t j = 0; j < MORSE_DICTIONARY_SIZE; j++) {

            if (strcmp(morseDictionary[j].code, symbol) == 0) {

                if (output_index + 1 >= output_size) {
                    output[0] = '\0';
                    return 0;
                }

                output[output_index++] = morseDictionary[j].character;
                found = 1;
                break;
            }
        }

        // Invalid Morse sequence.
        if (!found) {
            output[0] = '\0';
            return 0;
        }
    }

    output[output_index] = '\0';

    return 1;
}

int text_to_morse(const char *text, char *output, size_t output_size)
{
    size_t output_index = 0;
    int first_letter = 1;
    int new_word = 0;

    if (text == NULL || output == NULL || output_size == 0)
    {
        return 0;
    }

    output[0] = '\0';

    for (size_t i = 0; text[i] != '\0'; i++)
    {

        char letter = text[i];

        // Convert lowercase letters into uppercase.
        if (letter >= 'a' && letter <= 'z')
        {
            letter = letter - ('a' - 'A');
        }

        // Remember when a new word starts.
        if (letter == ' ')
        {
            if (!first_letter)
            {
                new_word = 1;
            }
            continue;
        }

        // Find the character in the Morse dictionary.
        const char *morse_code = NULL;

        for (size_t j = 0; j < MORSE_DICTIONARY_SIZE; j++)
        {
            if (morseDictionary[j].character == letter)
            {
                morse_code = morseDictionary[j].code;
                break;
            }
        }

        // Character was not found in the dictionary.
        if (morse_code == NULL)
        {
            output[0] = '\0';
            return 0;
        }

        // Add a separator before the next Morse letter.
        if (!first_letter)
        {

            if (new_word)
            {
                // Two spaces between words.
                if (output_index + 2 >= output_size)
                {
                    output[0] = '\0';
                    return 0;
                }
                output[output_index++] = ' ';
                output[output_index++] = ' ';
            }
            else
            {
                // One space between letters.
                if (output_index + 1 >= output_size)
                {
                    output[0] = '\0';
                    return 0;
                }

                output[output_index++] = ' ';
            }
        }
        // Check whether the output buffer has enough space.
        size_t code_length = strlen(morse_code);

        if (output_index + code_length >= output_size)
        {
            output[0] = '\0';
            return 0;
        }

        // Copy the Morse code into the output.
        for (size_t k = 0; k < code_length; k++)
        {
            output[output_index++] = morse_code[k];
        }

        first_letter = 0;
        new_word = 0;
    }

    output[output_index] = '\0';

    return 1;
}
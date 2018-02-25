void PrintByte(char *OutputString,
               char *PromptString,
               char ByteVar)
/* This function prints the unsigned value of a single byte variable. First,
   PromptString is copied to OutputString, and then unsigned decimal value
   of ByteVar is appended as a 3-character string. This function can replace
   sprintf() for limited applications. For example,
     sprintf(LCDtext, "%3d", ADCout);
     sprintf(LCDtext, "Atten=%3d", Atten);
   can be replaced by:
     PrintByte(LCDtext, "", ADCout);
     PrintByte(LCDtext, "Atten=", Atten);
*/
{
  char *pOut, *pPrompt;
  unsigned char Number, Digit;

// Copy prompt text to output:
  pPrompt = PromptString;
  pOut = OutputString;
  while (*pPrompt != 0x00)
  { *pOut = *pPrompt;
    pPrompt ++;
    pOut ++;
  };
// Determine 100's digit:
  Number = (unsigned char)ByteVar;
  if (Number < 100)
  { Digit = 0;
    *pOut = ' ';
  }
  else
  { Digit = Number / 100;
    *pOut = Digit + '0';
    Number -= Digit * 100;
  }
  pOut ++;
// Determine 10's digit:
  if (Number < 10)
  { if (Digit == 0)
      *pOut = ' ';
    else
      *pOut = '0';
  }
  else
  { Digit = Number / 10;
    *pOut = Digit + '0';
    Number -= Digit * 10;
  }
  pOut ++;
// Whatever left in Number is 1's digit:
  *pOut = Number + '0';
  pOut ++;
// Terminate the output string:
  *pOut = 0x00;
}

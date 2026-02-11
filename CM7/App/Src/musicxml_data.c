#include <stddef.h>

/* Plain-text MusicXML (UNCOMPRESSED .musicxml only) */
const char g_musicxml[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
"<!DOCTYPE score-partwise PUBLIC\n"
"    \"-//Recordare//DTD MusicXML 3.1 Partwise//EN\"\n"
"    \"http://www.musicxml.org/dtds/partwise.dtd\">\n"
"<score-partwise version=\"3.1\">\n"
"  <work>\n"
"    <work-title>Smoke on the Water</work-title>\n"
"  </work>\n"
"  <identification>\n"
"    <encoding>\n"
"      <software>MuseScore</software>\n"
"    </encoding>\n"
"  </identification>\n"
"  <part-list>\n"
"    <score-part id=\"P1\">\n"
"      <part-name>Guitar</part-name>\n"
"    </score-part>\n"
"  </part-list>\n"
"  <part id=\"P1\">\n"
"    <!-- measures + notes go here -->\n"
"  </part>\n"
"</score-partwise>\n";

const size_t g_musicxml_len = sizeof(g_musicxml) - 1;

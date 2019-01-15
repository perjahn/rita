; Rita by Per Jahn

rita		segment	byte public
		assume	cs:rita, ds:rita

		org	100h

start:		mov	sp, 1000h	; 256d (PSP) + 3840d bytes

		call	init
		jc	quit

drawline:	mov	ax, 0003	; ax=0003
		int	33h		; Mus knappar och position
		shr	cx, 1		; Halvera cx
		or	bx, bx		; Är bx=0000
		jz	nobutt		; Hoppa om ingen knapp trycks

		cmp	oldbutt, bx	; Testa om knapp har ändrats
		jne	draw
		cmp	oldx, cx	; Testa om xpos har ändrats
		jne	draw
		cmp	oldy, dx	; Testa om ypos har ändrats
		je	nobutt

draw:		mov	al, 02
		int	33h		; Göm musen

		cmp	bx, 4
		jne	ok
		call	setup
		jmp	cont

ok:		push	bx
		push	oldy
		push	oldx
		push	dx
		push	cx
		mov	oldx, cx
		mov	oldy, dx
		mov	oldbutt, bx
		call	_Line13		; Rita linje
		add	sp, 10d

cont:		mov	ax, 0001	; ax=0001
		int	33h		; Visa musen

		jmp	nosave

nobutt:		mov	oldx, cx
		mov	oldy, dx
		mov	oldbutt, bx

nosave:		mov	ah, 01		; ah=01
		int	16h		; Testa tangenttryck
		jz	drawline	; Hoppa om ingen tangent trycks
		xor	ah, ah		; ah=00
		int	16h		; Ta tangenttryck

		cmp	al, 27d
		je	quit

		cmp	ah, 59d
		jne	next1
		call	F1
		jmp	drawline

next1:		cmp	ah, 68d
		jne	next2
		call	F10
next2:		jmp	drawline


quit:		mov	ax, 0002	; ax=0002
		int	33h		; Göm musen
		mov	al, 03h		; ah=00, al=03
		int	10h		; Byt till 80x25x16
		int	20h		; Avsluta

oldx		dw	0
oldy		dw	0
oldbutt		dw	0
buf		dw	0		; Adress till 64K buffer

;***********************************************************

init		proc	near

		xor	ax, ax		; ax=0000
		int	33h		; Testa mus
		or	ax, ax		; Är ax=0000
		jz	aquit2		; Avsluta om mus inte finns

aresize:	mov	ah, 4Ah		; ah=4Ah
		mov	bx, 100h	; 256d (PSP) + 3840d bytes
		push	cs
		pop	es		; es=cs
		int	21h		; Ändra programminne
		jc	aquit		; Avsluta om fel

aalloc:		mov	ah, 48h		; ah=48h
		mov	bx, 4000d	; bx=4000d
		int	21h		; Allokera 4000 paragraphs, 64000 bytes
		jc	aquit		; Avsluta om för lite minne
		mov	buf, ax		; Flytta pekaren till buf

		mov	ax, 0013h	; ah=00, al=13
		int	10h		; Byt till 320x200x256

		mov	ax, 0A000h
		mov	es, ax
		mov	cx, 8000h	; cx = 32768d
		mov	ax, 0F0Fh
		rep	stosw

		mov	ax, 0009h
		xor	bx, bx
		mov	cx, bx
		mov	dx, offset musbuf
		push	cs
		pop	es
		int	33h		; Ändra musmarkör

		mov	al, 01		; ax=0001
		int	33h		; Visa musen

aquit:		ret

aquit2:		stc
		ret

musbuf		db	0FFh, 03Fh, 0FFh, 01Fh, 0FFh, 00Fh, 0FFh, 007h
		db	0FFh, 003h, 0FFh, 001h, 0FFh, 000h, 07Fh, 000h
		db	03Fh, 000h, 01Fh, 000h, 0FFh, 001h, 0FFh, 010h
		db	0FFh, 030h, 07Fh, 0F8h, 07Fh, 0F8h, 07Fh, 0FCh
		db	000h, 000h, 000h, 040h, 000h, 060h, 000h, 070h
		db	000h, 078h, 000h, 07Ch, 000h, 07Eh, 000h, 07Fh
		db	080h, 07Fh, 000h, 07Ch, 000h, 06Ch, 000h, 046h
		db	000h, 006h, 000h, 003h, 000h, 003h, 000h, 000h

init		endp

;***********************************************************

F1		proc	near

		ret

F1		endp

;***********************************************************

F10		proc	near

		mov	ax, 0002	; ax=0002
		int	33h		; Göm musen

		xor	si, si
		mov	di, si
		mov	cx, 2560d

		mov	es, buf
		mov	ax, 0A000h
		mov	ds, ax

		cmp	cs:pal, 0
		jz	fpal

		push	ds
		push	es
		pop	ds
		pop	es

fpal:		rep	movsw		; [es:di] = [ds:si]

		cmp	cs:pal, 0
		jnz	fquit

		mov	ax, cx		; ax = 0000h
		push	ds
		pop	es		; es = A000h
		mov	di, ax		; di = 0000h

flop:		mov	cx, 5
		rep	stosb		; [es:di] = al
		inc	al

		test	al, 00111111b
		jnz	flop

		sub	al, 64d
		inc	ah
		test	ah, 00000011b
		jnz	flop

		add	al, 64d
		or	al, al
		jnz	flop


fquit:		push	cs
		pop	ds

		mov	ax, 0001	; ax=0001
		int	33h		; Visa musen

		not	pal

		ret

pal		db	00h

F10		endp

;***********************************************************

ARGx1		EQU	word ptr [bp+4]	; stackramsadressering
ARGy1		EQU	word ptr [bp+6]
ARGx2		EQU	word ptr [bp+8]
ARGy2		EQU	word ptr [bp+10]
ARGn		EQU	byte ptr [bp+12]
VARincr1	EQU	word ptr [bp-2]
VARincr2	EQU	word ptr [bp-4]
VARroutine	EQU	word ptr [bp-6]

BytesPerLine	EQU	320

_Line13		PROC	near

		push	bp		; spara registerinnehåll vid anropet
		mov	bp,sp
		sub	sp,6		; stackutrymme för lokala variabler
		push	si
		push	di

; testa om lodrät linje

		mov	si,BytesPerLine	; första y-steg

		mov	cx,ARGx2
		sub	cx,ARGx1	; CX := x2 - x1
		jz	VertLine13	; hoppa om lodrät linje

; tvinga x1 < x2

		jns	L01		; hoppa om x2 > x1

		neg	cx		; CX := x1 - x2

		mov	bx,ARGx2	; skifta x1 och x2
		xchg	bx,ARGx1
		mov	ARGx2,bx

		mov	bx,ARGy2	; skifta y1 och y2
		xchg	bx,ARGy1
		mov	ARGy2,bx

; beräkna dy = ABS(y2-y1)

L01:		mov	bx,ARGy2
		sub	bx,ARGy1	; BX := y2 - y1
		jz	HorizLine13	; hoppa om vågrät linje

		jns	L03		; hoppa om lutningen positiv

		neg	bx		; BX := y1 - y2
		neg	si		; negera y-steg

; välj lämplig rutin för linjens lutning

L03:		push	si		; spara y-steg

		mov	VARroutine,offset LoSlopeLine13
		cmp	bx,cx
		jle	L04		; hoppa om dy <= dx (lutning <= 1)
		mov	VARroutine,offset HiSlopeLine13
		xchg	bx,cx		; skifta dy och dx

; beräkna första beslutsvariabel och steg

L04:		shl	bx,1		; BX := 2 * dy
		mov	VARincr1,bx	; incr1 := 2 * dy
		sub	bx,cx
		mov	si,bx		; SI := d = 2 * dy - dx
		sub	bx,cx
		mov	VARincr2,bx	; incr2 := 2 * (dy - dx)

; beräkna första bildelementadress

		push	cx		; spara detta register
		mov	ax,ARGy1	; AX := y
		mov	bx,ARGx1	; BX := x
		call	PixelAddr13	; ES:BX -> buffert

		mov	di,bx		; ES:DI -> buffert

		pop	cx		; återställ detta register
		inc	cx		; CX := antal bildelement som skall ritas

		pop	bx		; BX := y-steg
		jmp	VARroutine	; hoppa till rätt rutin för lutningen

; rutin för lodräta linjer

VertLine13:	mov	ax,ARGy1	; AX := y1
		mov	bx,ARGy2	; BX := y2
		mov	cx,bx
		sub	cx,ax		; CX := dy
		jge	L31		; hoppa om dy >= 0

		neg	cx		; tvinga dy >= 0
		mov	ax,bx		; AX := y2

L31:		inc	cx		; CX := antal bildelement som skall ritas
		mov	bx,ARGx1	; BX := x
		push	cx		; spara detta register
		call	PixelAddr13	; ES:BX -> videobuffert
		pop	cx

		mov	di,bx		; ES:DI -> videobuffert
		dec	si		; SI := byte/rad - 1

		mov	al,ARGn		; AL := bildelementvärde

L32:		stosb			; sätt bildelementvärdet i bufferten
		add	di,si		; stega till nästa rad
		loop	L32

		jmp	Lexit

; rutin för vågräta linjer (lutningen = 0)

HorizLine13:
		push	cx		; spara CX
		mov	ax,ARGy1
		mov	bx,ARGx1
		call	PixelAddr13	; ES:BX -> videobuffert
		mov	di,bx		; ES:DI -> buffert

		pop	cx
		inc	cx		; CX := antal bildelement som skall ritas

		mov	al,ARGn		; AL := bildelementvärde

		rep	stosb		; uppdatera videobufferten

		jmp	short Lexit

; rutin för dy <= dx (lutning <= 1)	; ES:DI -> videobuffert
					; BX = y-steg
					; CX = antal bildel. att rita
					; SI = beslutsvariabel

LoSlopeLine13:
		mov	al,ARGn		; AL := bildelementvärde

L11:		stosb			; lagra bildelement, stega x

		or	si,si		; testa tecknet hos d
		jns	L12		; hoppa om d >= 0

		add	si,VARincr1	; d := d + incr1
		loop	L11
		jmp	short Lexit

L12:		add	si,VARincr2	; d := d + incr2
		add	di,bx		; stega y
		loop	L11
		jmp	short Lexit

; rutin för dy > dx (lutning > 1)	; ES:DI -> videobuffert
					; BX := y-steg
					; CX := antal bildel. att rita
					; SI := beslutsvariabel
HiSlopeLine13:
		mov	al,ARGn		; AL := bildelementvärde

L21:		stosb			; uppdatera nästa bildelement, stega x

		add	di,bx		; stega y

L22:		or	si,si		; testa tecknet hos d
		jns	L23		; hoppa om d >= 0

		add	si,VARincr1	; d := d + incr1
		dec	di		; stega ner x (redan uppstegat av stosb)

		loop	L21
		jmp	short Lexit

L23:		add	si,VARincr2	; d := d + incr2
		loop	L21

Lexit:		pop	di		; återställ registerinnehåll
		pop	si
		mov	sp,bp
		pop	bp
		ret			; och hoppa tillbaka

_Line13		ENDP

;***********************************************************

OriginOffset	EQU	0		; offset i byte för (0,0)
VideoBufferSeg	EQU	0A000h

PixelAddr13	PROC	near

		xchg	ah,al		; AX := 256*y
		add	bx,ax		; BX := 256*y + x
		shr	ax,1
		shr	ax,1		; AX := 64*y
		add	bx,ax		; BX := 320*y + x

		add	bx,OriginOffset	; BX := offset i byte i videobufferten
		mov	ax,VideoBufferSeg
		mov	es,ax		; ES:BX := adress i byte för bildelementet
		ret

PixelAddr13	ENDP

;***********************************************************
; ah = fillcolor
; al = bgcolor
; bx = addr
; ds = A000h

fill		proc	near

		cmp	al, [bx]
		jne	fiquit
		mov	[bx], ah

		dec	bx
		call	fill
		add	bx, 2d
		call	fill
		sub	bx, 321d
		call	fill
		add	bx, 640d
		call	fill
		sub	bx, 320d

fiquit:		ret

fill		endp

;PixelFill( int x, int y )
;{
;	int v;
;	v = ReadPixel( x, y );
;	if( (v!=FillValue) && (v!=BorderValue) )
;	{
;		SetPixel( x, y, FillValue );
;		PixelFill( x-1, y );
;		PixelFill( x+1, y );
;		PixelFill( x, y-1 );
;		PixelFill( x, y+1 );
;	}
;}

setup		proc	near

		mov	ax, dx
		mov	bx, cx
		call	PixelAddr13	; [es:di] = pixel

		mov	ah, bl		; ah = fillcolor
		mov	al, [es:di]	; al = bgcolor
		mov	bx, di		; bx = addr
		push	es
		pop	ds		; ds = A000h
		call	fill

		ret

setup		endp

;***********************************************************

ARGxc		EQU	word ptr [bp+4]	; stackramsadressering
ARGyc		EQU	word ptr [bp+6]
ARGa		EQU	word ptr [bp+8]
ARGb		EQU	word ptr [bp+10]
ARGn		EQU	byte ptr [bp+12]

ULAddr		EQU	word ptr [bp-2]
URAddr		EQU	word ptr [bp-4]
LLAddr		EQU	word ptr [bp-6]
LRAddr		EQU	word ptr [bp-8]
LMask		EQU	byte ptr [bp-10]
RMask		EQU	byte ptr [bp-12]

VARd		EQU	word ptr [bp-16]
VARdx		EQU	word ptr [bp-20]
VARdy		EQU	word ptr [bp-24]
Asquared	EQU	word ptr [bp-28]
Bsquared	EQU	word ptr [bp-32]
TwoAsquared	EQU	word ptr [bp-36]
TwoBsquared	EQU	word ptr [bp-40]

RMWbits		EQU	0		; bitar för läs-modifiera-skriv
;BytesPerLine	EQU	80

_Ellipse10	PROC	near

		push	bp		; spara registerinnehåll vid anropet
		mov	bp,sp
		sub	sp,40		; sätt upp stackramen
		push	si
		push	di

; sätt grafiklägesregistret i grafikstyrenheten

		mov	dx,3CEh		; DX := port för grafikstyrenheten
		mov	ax,0005h	; AL := nummer på grafiklägeregistret
					; AH := Skrivläge 0 (bitarna 0,1)
		out	dx,ax		;	Läsläge 0 (bit 4)

; sätt registret Data Rotate/Function Select

		mov	ah,RMWbits	; AH := bitar för läs-modifiera-skriv
		mov	al,3		; AL := reg Data Rotate/Function Select
		out	dx,ax

; sätt registren Set/Reset och Enable Set/Reset

		mov	ah, ARGn	; AH := bildelementvärde
		mov	al,0		; AL := nummer på reg Set/Reset
		out	dx,ax

		mov	ax,0F01h	; AH := värde för Enable Set/Reset (alla
					;  bitplan aktiverade)
		out	dx,ax		; AL := nummer på reg Enable Set/Reset

; sätt upp konstanterna

		mov	ax,ARGa
		mul	ax
		mov	Asquared,ax
		mov	Asquared+2,dx	; a^2
		shl	ax,1
		rcl	dx,1
		mov	TwoAsquared,ax
		mov	TwoAsquared+2,dx ; 2*a^2

		mov	ax,ARGb
		mul	ax
		mov	Bsquared,ax
		mov	Bsquared+2,dx	; b^2
		shl	ax,1
		rcl	dx,1
		mov	TwoBsquared,ax
		mov	TwoBsquared+2,dx ; 2*b^2

; startvärde för adress i bufferten och bitmask

		mov	ax,BytesPerLine	; AX := radlängd i videobufferten
		mul	ARGb		; AX := relativ offset i byte för b
		mov	si,ax
		mov	di,ax

		mov	ax,ARGyc	; AX := yc
		mov	bx,ARGxc	; BX := xc
		call	PixelAddr10	; AH := bitmask
					; ES:BX -> buffert
					; CL := antal bitar vänsterskift
		mov	ah,1
		shl	ah,cl		; AH := bitmask för första bildelement
		mov	LMask,ah
		mov	RMask,ah

		add	si,bx		; SI := offset för (0,b)
		mov	ULAddr,si
		mov	URAddr,si
		sub	bx,di		; AX := offset för (0,-b)
		mov	LLAddr,bx
		mov	LRAddr,bx

; startvärde för beslutsvariabeler

		xor	ax,ax
		mov	VARdx,ax
		mov	VARdx+2,ax	; dx = 0

		mov	ax,TwoAsquared
		mov	dx,TwoAsquared+2
		mov	cx,ARGb
		call	LongMultiply	; gör 32-bitars gånger 16-bitars multiplikation
		mov	VARdy,ax
		mov	VARdy+2,dx	; dy = TwoAsquared * b

		mov	ax,Asquared
		mov	dx,Asquared+2	; DX:AX = Asquared
		sar	dx,1
		rcr	ax,1
		sar	dx,1
		rcr	ax,1		; DX:AX = Asquared/4

		add	ax,Bsquared
		adc	dx,Bsquared+2	; DX:AX = Bsquared + Asquared/4
		mov	VARd,ax
		mov	VARd+2,dx

		mov	ax,Asquared
		mov	dx,Asquared+2
		mov	cx,ARGb
		call	LongMultiply	; DX:AX = Asquared*b
		sub	VARd,ax
		sbb	VARd+2,dx	; d = Bsquared - Asquared*b + Asquared/4

; gå runt tills dy/dx >= -1

		mov	bx,ARGb		; BX := startvärde för y-koordinat

		xor	cx,cx		; CH := 0 (första y-steg)
					; CL := 0 (första x-steg)
eL10:		mov	ax,VARdx
		mov	dx,VARdx+2

		sub	ax,VARdy
		sbb	dx,VARdy+2
		jne	eL20		; hoppa om dx>=dy

		call	Set4Pixels

		mov	cx,1		; CH := 0 (y-steg)
					; CL := 0 (x-steg)
		cmp	VARd+2,0
		js	eL11		; hoppa om d < 0

		mov	cx,1		; stega i y-riktningen
		dec	bx		; minska aktuell y-koordinat

		mov	ax,VARdy
		mov	dx,VARdy+2
		sub	ax,TwoAsquared
		sbb	dx,TwoAsquared+2 ; DX:AX := dy - TwoAsquared
		mov	VARdy,ax
		mov	VARdy+2,dx	; dy -= TwoAsquared

		sub	VARd,ax
		sbb	VARdy+2,dx	; d -= dy

eL11:		mov	ax,VARdx
		mov	dx,VARdx+2
		add	ax,TwoBsquared
		adc	dx,TwoBsquared+2 ; DX:AX := dx + TwoBsquared
		mov	VARdx,ax
		mov	VARdx+2,dx	; dx += TwoBsquared

		add	ax,Bsquared
		adc	dx,Bsquared+2	; DX:AX := dx + Bsquared
		sub	VARd,ax
		sbb	VARd+2,dx	; d += dx + Bsquared

		jmp	eL10

;
; rita bildelement från aktuella (x,y) tills y < 0
;

; startvärde för buffertadress och bitmask

eL20:		push	bx		; spara aktuell y-koordinat
		push	cx		; spara x- och y-steg

		mov	ax,Asquared
		mov	dx,Asquared+2
		sub	ax,Bsquared
		sbb	dx,Bsquared+2	; DX:AX := Asquared-Bsquared

		mov	bx,ax
		mov	cx,dx		; CX:BX := (Asquared-Bsquared)

		sar	dx,1
		rcr	ax,1		; DX:AX := (Asquared-Bsquared)/2
		add	ax,bx
		adc	dx,cx		; DX:AX := 3*(Asquared-Bsquared)/2

		sub	ax,VARdx
		sbb	dx,VARdx+2
		sub	ax,VARdy
		sbb	dx,VARdy	; DX:AX := 3*(Asquared-Bsquared)/2 - (dx+dy)

		sar	dx,1
		rcr	ax,1		; DX:AX :=
					;  ( 3*(Asquared-Bsquared)/2 - (dx+dy) )/2
		add	VARd,ax
		adc	VARd+2,dx	; uppdatera d

; gå runt tills y < 0

		pop	cx		; CH,CL := y- och x-steg
		pop	bx		; BX := y

eL21:		call	Set4Pixels

		mov	cx,100h		; CH := 1 (y-steg)
					; CL := 0 (x-steg)
		cmp	VARd+2,0
		jns	eL22		; hoppa om d >= 0

		mov	cl,1		; steg i x-riktningen

		mov	ax,VARdx
		mov	dx,VARdx+2
		add	ax,TwoBsquared
		adc	dx,TwoBsquared+2 ; DX:AX := dx + TwoBsquared
		mov	VARdx,ax
		mov	VARdx+2,dx	; dx += TwoBsquared

		add	VARd,ax
		adc	VARdx+2,dx	; d += dx

eL22:		mov	ax,VARdy
		mov	dx,VARdy+2
		sub	ax,TwoAsquared
		sbb	dx,TwoAsquared+2 ; DX:AX := dy - TwoAsquared
		mov	VARdy,ax
		mov	VARdy+2,dx	; dy -= TwoAsquared

		sub	ax,Asquared
		sbb	dx,Asquared+2	; DX:AX := dy - Asquared
		sub	VARd,ax
		sbb	VARd+2,dx	; d += Asquared - dy

		dec	bx		; minska y
		jns	eL21		; gå runt om >= 0

; återställ standardvärden för register i grafikstyrenheten

eLexit:		mov	ax,0FF08h	; standardvärde för bitmaskreg
		mov	dx,3CEh
		out	dx,ax

		mov	ax,0003		; standardvärde för Function Select
		out	dx,ax

		mov	ax,0001		; standardvärde för Enable Set/Reset
		out	dx,ax

		pop	di		; återställ register och hoppa tillbaka
		pop	si
		mov	sp,bp
		pop	bp
		ret

_Ellipse10	ENDP

;***********************************************************

Set4Pixels	PROC	near		; Anropa med: CH := y-steg (0, -1)
					;	      CL := x-steg (0, 1)

		push	ax		; spara dessa reg
		push	bx
		push	dx

		mov	dx,3CEh		; DX := port för grafikstyrenheten

		xor	bx,bx		; BX := 0
		test	ch,ch
		jz	eL30		; hoppa om y-steg = 0

		mov	bx,BytesPerLine	; BX := positivt steg
		neg	bx		; BX := negativt steg

eL30:		mov	al,8		; AL := nummer på bitmaskreg

; bildelement i (cx-y,yc+y) och (xc-x,yc-y)

		xor	si,si		; SI := 0
		mov	ah,LMask

		rol	ah,cl		; AH := bitmask roterad vågrätt
		rcl	si,1		; SI := 1 om bitmasken roterad ett varv
		neg	si		; SI := 0 eller -1

		mov	di,si		; SI,DI := steg vågrätt vänster

		add	si,ULAddr	; SI := övre vänster adr + vågrätt steg
		add	si,bx		; SI := nya övre vänster adr
		add	di,LLAddr
		sub	di,bx		; DI := nya nedre vänster adr

		mov	LMask,ah	; uppdatera dessa variabler
		mov	ULAddr,si
		mov	LLAddr,di

		out	dx,ax		; uppdatera bitmaskregistret

		mov	ch,es:[si]	; uppdatera övre vänster bildelement
		mov	es:[si],ch
		mov	ch,es:[di]	; uppdatera nedre vänster bildelement
		mov	es:[di],ch

; bildelement i (xc+x,yc+y) och (xc+x,yc-y)

		xor	si,si		; SI := 0
		mov	ah,RMask

		ror	ah,cl		; AH := bitmask roterad vågrätt
		rcl	si,1		; SI := 1 om bitmask roterad ett varv

		mov	di,si		; SI,DI := steg höger vågrätt

		add	si,URAddr	; SI := övre högra adr + vågrätt steg
		add	si,bx		; SI := nya övre höger adr
		add	di,LRAddr
		sub	di,bx		; DI := nya nedre höger adr

		mov	RMask,ah	; uppdatera dessa variabler
		mov	URAddr,si
		mov	LRAddr,di

		out	dx,ax		; uppdatera bitmaskregistret

		mov	ch,es:[si]	; uppdatera övre höger bildelement
		mov	es:[si],ch
		mov	ch,es:[di]	; uppdatera nedre höger bildelement
		mov	es:[di],ch

		pop	dx		; återställ dessa reg
		pop	bx
		pop	ax
		ret

Set4Pixels	ENDP

;***********************************************************

LongMultiply	PROC	near		; Anropas med: DX = ul (höga ordet
					;		av 32-bitars tal)
					;		AX = u2 (låga ordet)
					;		CX = vl (16-bitars tal)
					; Returvärde:	DX:AX = 32-bitars resultat
		push	ax		; spara u2
		mov	ax,dx		; AX := ul
		mul	cx		; AX := höga ordet i resultatet
		xchg	ax,cx		; AX := vl
		pop	dx		; DX := u2
		mul	dx		; AX := låga ordet i resultatet
					; DX := minnessiffra
		add	dx,cx		; CX := höga ordet i resultatet
		ret

LongMultiply	ENDP

;***********************************************************

;BytesPerLine	EQU	80		; antal byte i en horisontell linje
OriginOffset	EQU	0		; offset i byte för (0,0)
VideoBuffertSeg	EQU	0A000h

PixelAddr10	PROC	near

		mov	cl, bl		; CL = låga byte hos x
		push	dx		; spara dx
		mov	dx,BytesPerLine	; AX = y * BytesPerLine
		mul	dx
		pop	dx
		shr	bx,1
		shr	bx,1
		shr	bx,1		; BX = x/8
		add	bx,ax		; BX = y*BytesPerLine + x/8
		add	bx,OriginOffset	; BX = offset i byte i bildelement
		mov	ax,VideoBuffertSeg
		mov	es,ax		; ES:BX = adress i byte för bildelementet
		and	cl,7		; CL = x & 7
		xor	cl,7		; CL = antal bitar vänsterskift
		mov	ah,1		; AH = oskiftad bitmask
		ret

PixelAddr10	ENDP

;***********************************************************

rita		ends
		end	start

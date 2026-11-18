package com.example.chess_clock_app.views

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import com.example.chess_clock_app.chess.Move
import com.example.chess_clock_app.chess.PieceType
import com.example.chess_clock_app.chess.Position
import com.example.chess_clock_app.chess.Side
import com.example.chess_clock_app.chess.fileOf
import com.example.chess_clock_app.chess.rankOf
import com.example.chess_clock_app.chess.squareOf
import kotlin.math.min

// Draws a Position and lets the user pick a move: tap a piece, then tap one of the marked squares.
// The view never changes the game itself; it reports the move through onMoveRequested and
// the ViewModel decides what happens, then hands back the new position.
class ChessBoardView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    var position: Position = Position.start()
        set(value) {
            field = value
            clearSelection()
            invalidate()
        }

    // Highlighted so you can see which move led to the current position
    var lastMove: Move? = null
        set(value) {
            field = value
            invalidate()
        }

    var onMoveRequested: ((from: Int, to: Int, isPromotion: Boolean) -> Unit)? = null

    private var selectedSquare: Int? = null
    private var selectedMoves: List<Move> = emptyList()

    private val lightSquarePaint = fillPaint(Color.rgb(238, 238, 210))
    private val darkSquarePaint = fillPaint(Color.rgb(118, 150, 86))
    private val lastMovePaint = fillPaint(Color.argb(110, 255, 235, 59))
    private val selectedPaint = fillPaint(Color.argb(140, 255, 193, 7))
    private val targetPaint = fillPaint(Color.argb(90, 0, 0, 0))
    private val labelPaint = fillPaint(Color.argb(160, 0, 0, 0))
    private val pieceFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { textAlign = Paint.Align.CENTER }
    private val pieceOutlinePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textAlign = Paint.Align.CENTER
        style = Paint.Style.STROKE
        color = Color.BLACK
    }

    // Always as tall as it is wide
    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val width = MeasureSpec.getSize(widthMeasureSpec)
        val height = MeasureSpec.getSize(heightMeasureSpec)
        val size = if (MeasureSpec.getMode(heightMeasureSpec) == MeasureSpec.UNSPECIFIED) {
            width
        } else {
            min(width, height)
        }
        setMeasuredDimension(size, size)
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val squareSize = width / 8f
        pieceFillPaint.textSize = squareSize * 0.8f
        pieceOutlinePaint.textSize = squareSize * 0.8f
        pieceOutlinePaint.strokeWidth = squareSize * 0.025f
        labelPaint.textSize = squareSize * 0.2f

        for (square in 0 until 64) {
            val left = fileOf(square) * squareSize
            val top = (7 - rankOf(square)) * squareSize
            val isLight = (fileOf(square) + rankOf(square)) % 2 == 1

            canvas.drawRect(left, top, left + squareSize, top + squareSize,
                if (isLight) lightSquarePaint else darkSquarePaint)
            if (square == lastMove?.from || square == lastMove?.to) {
                canvas.drawRect(left, top, left + squareSize, top + squareSize, lastMovePaint)
            }
            if (square == selectedSquare) {
                canvas.drawRect(left, top, left + squareSize, top + squareSize, selectedPaint)
            }

            // Coordinates along the left and bottom edges
            if (fileOf(square) == 0) {
                canvas.drawText("${rankOf(square) + 1}", left + squareSize * 0.05f,
                    top + squareSize * 0.22f, labelPaint)
            }
            if (rankOf(square) == 0) {
                canvas.drawText("${'a' + fileOf(square)}", left + squareSize * 0.82f,
                    top + squareSize * 0.95f, labelPaint)
            }

            position[square]?.let { piece ->
                val glyph = GLYPHS.getValue(piece.type)
                val x = left + squareSize / 2
                // Vertically center the glyph in the square
                val y = top + squareSize / 2 -
                    (pieceFillPaint.descent() + pieceFillPaint.ascent()) / 2
                pieceFillPaint.color = if (piece.side == Side.WHITE) Color.WHITE else Color.BLACK
                canvas.drawText(glyph, x, y, pieceFillPaint)
                if (piece.side == Side.WHITE) canvas.drawText(glyph, x, y, pieceOutlinePaint)
            }
        }

        // A dot on every square the selected piece can move to
        for (target in selectedMoves.map { it.to }.distinct()) {
            val centerX = fileOf(target) * squareSize + squareSize / 2
            val centerY = (7 - rankOf(target)) * squareSize + squareSize / 2
            canvas.drawCircle(centerX, centerY, squareSize * 0.15f, targetPaint)
        }
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.action) {
            MotionEvent.ACTION_DOWN -> return true
            MotionEvent.ACTION_UP -> {
                val squareSize = width / 8f
                val file = (event.x / squareSize).toInt()
                val rank = 7 - (event.y / squareSize).toInt()
                if (file in 0..7 && rank in 0..7) onSquareTapped(squareOf(file, rank))
                performClick()
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean = super.performClick()

    private fun onSquareTapped(square: Int) {
        val from = selectedSquare
        val movesToSquare = selectedMoves.filter { it.to == square }

        // Second tap on a marked square: that's the move
        if (from != null && movesToSquare.isNotEmpty()) {
            clearSelection()
            invalidate()
            // A pawn reaching the last rank has 4 legal moves to that square, one per promotion piece
            onMoveRequested?.invoke(from, square, movesToSquare.any { it.promotion != null })
            return
        }

        // Tap on one of your own pieces selects it; tapping anything else clears the selection
        val piece = position[square]
        if (piece != null && piece.side == position.sideToMove && square != from) {
            selectedSquare = square
            selectedMoves = position.legalMoves().filter { it.from == square }
        } else {
            clearSelection()
        }
        invalidate()
    }

    private fun clearSelection() {
        selectedSquare = null
        selectedMoves = emptyList()
    }

    private fun fillPaint(color: Int) = Paint(Paint.ANTI_ALIAS_FLAG).apply { this.color = color }

    companion object {
        // Solid chess symbols for both sides; color comes from the paint.
        // ︎ asks for the text version, otherwise some phones draw the pawn as an emoji.
        private val GLYPHS = mapOf(
            PieceType.KING to "♚︎",
            PieceType.QUEEN to "♛︎",
            PieceType.ROOK to "♜︎",
            PieceType.BISHOP to "♝︎",
            PieceType.KNIGHT to "♞︎",
            PieceType.PAWN to "♟︎",
        )
    }
}

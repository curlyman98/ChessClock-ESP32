package com.example.chess_clock_app.chess

import com.example.chess_clock_app.chess.PieceType.BISHOP
import com.example.chess_clock_app.chess.PieceType.KING
import com.example.chess_clock_app.chess.PieceType.KNIGHT
import com.example.chess_clock_app.chess.PieceType.PAWN
import com.example.chess_clock_app.chess.PieceType.QUEEN
import com.example.chess_clock_app.chess.PieceType.ROOK
import kotlin.math.abs

enum class Side {
    WHITE, BLACK;

    val opponent: Side get() = if (this == WHITE) BLACK else WHITE
}

enum class PieceType(val letter: Char) {
    PAWN('P'), KNIGHT('N'), BISHOP('B'), ROOK('R'), QUEEN('Q'), KING('K')
}

data class Piece(val type: PieceType, val side: Side)

data class Move(val from: Int, val to: Int, val promotion: PieceType? = null)

// Squares are numbered 0..63: 0 = a1, 7 = h1, 8 = a2, ..., 63 = h8
fun squareOf(file: Int, rank: Int) = rank * 8 + file
fun fileOf(square: Int) = square % 8
fun rankOf(square: Int) = square / 8
fun squareName(square: Int) = "${'a' + fileOf(square)}${rankOf(square) + 1}"
private fun onBoard(file: Int, rank: Int) = file in 0..7 && rank in 0..7

// An immutable snapshot of the board. play() returns a new Position instead of changing this one,
// which makes stepping back and forth through a game trivial: just keep a list of positions.
class Position private constructor(
    private val board: Array<Piece?>,
    val sideToMove: Side,
    // FEN letters: K/Q = white may still castle king/queen side, k/q = same for black
    private val castling: Set<Char>,
    // The square a pawn can capture onto en passant; only set right after a pawn's double step
    private val enPassantSquare: Int?,
) {

    operator fun get(square: Int): Piece? = board[square]

    fun legalMoves(): List<Move> =
        pseudoLegalMoves().filter { !play(it).isKingAttacked(sideToMove) }

    fun isInCheck(): Boolean = isKingAttacked(sideToMove)

    fun play(move: Move): Position {
        val newBoard = board.copyOf()
        val piece = newBoard[move.from] ?: error("No piece on ${squareName(move.from)}")
        newBoard[move.from] = null

        // En passant: the captured pawn is not on the target square but next to the start square
        if (piece.type == PAWN && move.to == enPassantSquare && newBoard[move.to] == null) {
            newBoard[squareOf(fileOf(move.to), rankOf(move.from))] = null
        }

        // Castling: the king moves two squares, the rook jumps over it
        if (piece.type == KING && abs(fileOf(move.to) - fileOf(move.from)) == 2) {
            val rank = rankOf(move.from)
            val kingSide = fileOf(move.to) == 6
            val rookFrom = squareOf(if (kingSide) 7 else 0, rank)
            val rookTo = squareOf(if (kingSide) 5 else 3, rank)
            newBoard[rookTo] = newBoard[rookFrom]
            newBoard[rookFrom] = null
        }

        newBoard[move.to] = if (move.promotion != null) Piece(move.promotion, piece.side) else piece

        // Moving the king loses both rights; moving or capturing a rook on its corner loses one
        val newCastling = castling.toMutableSet()
        if (piece.type == KING) {
            newCastling -= if (piece.side == Side.WHITE) setOf('K', 'Q') else setOf('k', 'q')
        }
        for (square in listOf(move.from, move.to)) {
            when (square) {
                0 -> newCastling -= 'Q'
                7 -> newCastling -= 'K'
                56 -> newCastling -= 'q'
                63 -> newCastling -= 'k'
            }
        }

        val newEnPassant =
            if (piece.type == PAWN && abs(move.to - move.from) == 16) (move.from + move.to) / 2 else null

        return Position(newBoard, sideToMove.opponent, newCastling, newEnPassant)
    }

    // Standard algebraic notation as used in PGN: e4, Nbd7, exd5, O-O, e8=Q, Qh5+, Qxf7#
    fun toSan(move: Move): String {
        val san = sanWithoutCheck(move, legalMoves())
        val after = play(move)
        return when {
            !after.isInCheck() -> san
            after.legalMoves().isEmpty() -> "$san#"
            else -> "$san+"
        }
    }

    // Finds the legal move a SAN string describes, or null if it is not legal here
    fun moveFromSan(san: String): Move? {
        val wanted = san.trimEnd('+', '#', '!', '?').replace('0', 'O')
        val legal = legalMoves()
        return legal.firstOrNull { sanWithoutCheck(it, legal) == wanted }
    }

    private fun sanWithoutCheck(move: Move, legal: List<Move>): String {
        val piece = board[move.from] ?: error("No piece on ${squareName(move.from)}")

        if (piece.type == KING && abs(fileOf(move.to) - fileOf(move.from)) == 2) {
            return if (fileOf(move.to) == 6) "O-O" else "O-O-O"
        }

        val isCapture = board[move.to] != null || (piece.type == PAWN && move.to == enPassantSquare)
        val san = StringBuilder()

        if (piece.type == PAWN) {
            if (isCapture) san.append('a' + fileOf(move.from))
        } else {
            san.append(piece.type.letter)
            // If another piece of the same type can reach the same square, add its file and/or rank
            val rivals = legal.filter {
                it.to == move.to && it.from != move.from && board[it.from]?.type == piece.type
            }
            if (rivals.isNotEmpty()) {
                val sameFile = rivals.any { fileOf(it.from) == fileOf(move.from) }
                val sameRank = rivals.any { rankOf(it.from) == rankOf(move.from) }
                if (!sameFile) {
                    san.append('a' + fileOf(move.from))
                } else if (!sameRank) {
                    san.append(rankOf(move.from) + 1)
                } else {
                    san.append(squareName(move.from))
                }
            }
        }

        if (isCapture) san.append('x')
        san.append(squareName(move.to))
        if (move.promotion != null) san.append('=').append(move.promotion.letter)
        return san.toString()
    }

    // Moves that follow how pieces move, without checking whether they leave the own king in check
    private fun pseudoLegalMoves(): List<Move> {
        val moves = mutableListOf<Move>()
        for (from in 0 until 64) {
            val piece = board[from] ?: continue
            if (piece.side != sideToMove) continue
            when (piece.type) {
                PAWN -> addPawnMoves(from, moves)
                KNIGHT -> addSteps(from, KNIGHT_STEPS, moves)
                BISHOP -> addSlides(from, DIAGONALS, moves)
                ROOK -> addSlides(from, STRAIGHTS, moves)
                QUEEN -> addSlides(from, DIAGONALS + STRAIGHTS, moves)
                KING -> {
                    addSteps(from, KING_STEPS, moves)
                    addCastling(from, moves)
                }
            }
        }
        return moves
    }

    private fun addPawnMoves(from: Int, moves: MutableList<Move>) {
        val file = fileOf(from)
        val rank = rankOf(from)
        val direction = if (sideToMove == Side.WHITE) 1 else -1
        val startRank = if (sideToMove == Side.WHITE) 1 else 6
        val lastRank = if (sideToMove == Side.WHITE) 7 else 0

        fun add(to: Int) {
            if (rankOf(to) == lastRank) {
                listOf(QUEEN, ROOK, BISHOP, KNIGHT).forEach { moves += Move(from, to, it) }
            } else {
                moves += Move(from, to)
            }
        }

        // A pawn is never on the last rank, so one step forward is always on the board
        val oneStep = squareOf(file, rank + direction)
        if (board[oneStep] == null) {
            add(oneStep)
            val twoSteps = squareOf(file, rank + 2 * direction)
            if (rank == startRank && board[twoSteps] == null) moves += Move(from, twoSteps)
        }

        for (fileStep in listOf(-1, 1)) {
            if (file + fileStep !in 0..7) continue
            val to = squareOf(file + fileStep, rank + direction)
            val target = board[to]
            if ((target != null && target.side != sideToMove) || to == enPassantSquare) add(to)
        }
    }

    // Knight and king: jump to a fixed set of squares
    private fun addSteps(from: Int, steps: List<Pair<Int, Int>>, moves: MutableList<Move>) {
        for ((fileStep, rankStep) in steps) {
            val file = fileOf(from) + fileStep
            val rank = rankOf(from) + rankStep
            if (!onBoard(file, rank)) continue
            val to = squareOf(file, rank)
            val target = board[to]
            if (target == null || target.side != sideToMove) moves += Move(from, to)
        }
    }

    // Bishop, rook and queen: keep going in a direction until something is in the way
    private fun addSlides(from: Int, directions: List<Pair<Int, Int>>, moves: MutableList<Move>) {
        for ((fileStep, rankStep) in directions) {
            var file = fileOf(from) + fileStep
            var rank = rankOf(from) + rankStep
            while (onBoard(file, rank)) {
                val to = squareOf(file, rank)
                val target = board[to]
                if (target == null) {
                    moves += Move(from, to)
                } else {
                    if (target.side != sideToMove) moves += Move(from, to)
                    break
                }
                file += fileStep
                rank += rankStep
            }
        }
    }

    private fun addCastling(from: Int, moves: MutableList<Move>) {
        val rank = if (sideToMove == Side.WHITE) 0 else 7
        if (from != squareOf(4, rank)) return
        val enemy = sideToMove.opponent
        // You may not castle out of check
        if (isAttacked(from, enemy)) return

        val ownRook = Piece(ROOK, sideToMove)
        val kingSideRight = if (sideToMove == Side.WHITE) 'K' else 'k'
        val queenSideRight = if (sideToMove == Side.WHITE) 'Q' else 'q'

        // King side: f and g empty, rook on h, king does not pass through check
        if (kingSideRight in castling &&
            board[squareOf(7, rank)] == ownRook &&
            board[squareOf(5, rank)] == null && board[squareOf(6, rank)] == null &&
            !isAttacked(squareOf(5, rank), enemy)
        ) {
            moves += Move(from, squareOf(6, rank))
        }

        // Queen side: b, c and d empty, rook on a, king does not pass through check
        if (queenSideRight in castling &&
            board[squareOf(0, rank)] == ownRook &&
            board[squareOf(1, rank)] == null && board[squareOf(2, rank)] == null &&
            board[squareOf(3, rank)] == null &&
            !isAttacked(squareOf(3, rank), enemy)
        ) {
            moves += Move(from, squareOf(2, rank))
        }
        // Whether the destination square is attacked is checked by the legalMoves() filter
    }

    private fun isKingAttacked(side: Side): Boolean {
        val kingSquare = board.indexOfFirst { it == Piece(KING, side) }
        return kingSquare != -1 && isAttacked(kingSquare, side.opponent)
    }

    // Looks outwards from the square for pieces of the given side that could capture on it
    private fun isAttacked(square: Int, by: Side): Boolean {
        val file = fileOf(square)
        val rank = rankOf(square)

        fun isAttacker(file: Int, rank: Int, types: Set<PieceType>): Boolean {
            if (!onBoard(file, rank)) return false
            val piece = board[squareOf(file, rank)] ?: return false
            return piece.side == by && piece.type in types
        }

        // Pawns capture diagonally forward, so an attacking pawn sits one rank "behind" the square
        val pawnRank = if (by == Side.WHITE) rank - 1 else rank + 1
        if (isAttacker(file - 1, pawnRank, setOf(PAWN)) || isAttacker(file + 1, pawnRank, setOf(PAWN))) {
            return true
        }
        if (KNIGHT_STEPS.any { (f, r) -> isAttacker(file + f, rank + r, setOf(KNIGHT)) }) return true
        if (KING_STEPS.any { (f, r) -> isAttacker(file + f, rank + r, setOf(KING)) }) return true

        fun slidingAttack(directions: List<Pair<Int, Int>>, types: Set<PieceType>): Boolean {
            for ((fileStep, rankStep) in directions) {
                var f = file + fileStep
                var r = rank + rankStep
                while (onBoard(f, r)) {
                    val piece = board[squareOf(f, r)]
                    if (piece != null) {
                        if (piece.side == by && piece.type in types) return true
                        break
                    }
                    f += fileStep
                    r += rankStep
                }
            }
            return false
        }

        return slidingAttack(STRAIGHTS, setOf(ROOK, QUEEN)) ||
            slidingAttack(DIAGONALS, setOf(BISHOP, QUEEN))
    }

    companion object {
        private val KNIGHT_STEPS =
            listOf(1 to 2, 2 to 1, 2 to -1, 1 to -2, -1 to -2, -2 to -1, -2 to 1, -1 to 2)
        private val KING_STEPS =
            listOf(1 to 0, 1 to 1, 0 to 1, -1 to 1, -1 to 0, -1 to -1, 0 to -1, 1 to -1)
        private val STRAIGHTS = listOf(1 to 0, -1 to 0, 0 to 1, 0 to -1)
        private val DIAGONALS = listOf(1 to 1, 1 to -1, -1 to 1, -1 to -1)

        fun start(): Position {
            val board = arrayOfNulls<Piece>(64)
            val backRank = listOf(ROOK, KNIGHT, BISHOP, QUEEN, KING, BISHOP, KNIGHT, ROOK)
            for (file in 0..7) {
                board[squareOf(file, 0)] = Piece(backRank[file], Side.WHITE)
                board[squareOf(file, 1)] = Piece(PAWN, Side.WHITE)
                board[squareOf(file, 6)] = Piece(PAWN, Side.BLACK)
                board[squareOf(file, 7)] = Piece(backRank[file], Side.BLACK)
            }
            return Position(board, Side.WHITE, setOf('K', 'Q', 'k', 'q'), null)
        }
    }
}

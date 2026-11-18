package com.example.chess_clock_app.activities

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.TextView
import android.widget.Toast
import androidx.activity.OnBackPressedCallback
import androidx.activity.viewModels
import androidx.core.view.GravityCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.adapters.MoveAdapter
import com.example.chess_clock_app.chess.PieceType
import com.example.chess_clock_app.chess.Side
import com.example.chess_clock_app.utils.InputLimits
import com.example.chess_clock_app.utils.showEditDialog
import com.example.chess_clock_app.utils.formatDate
import com.example.chess_clock_app.utils.formatDateTime
import com.example.chess_clock_app.utils.limitLength
import com.example.chess_clock_app.utils.setEnabledLook
import com.example.chess_clock_app.utils.toLocalDate
import com.example.chess_clock_app.utils.withDate
import com.example.chess_clock_app.viewmodels.GameDetails
import com.example.chess_clock_app.viewmodels.GameEditViewModel
import com.example.chess_clock_app.views.ChessBoardView
import com.google.android.material.button.MaterialButtonToggleGroup
import com.google.android.material.datepicker.MaterialDatePicker
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.textfield.TextInputLayout
import kotlinx.coroutines.launch
import java.time.Instant
import java.time.LocalDate
import java.time.ZoneOffset

class GameEditActivity : BaseActivity() {

    companion object {
        // Key of the game to open. Started without it, the screen creates a new game.
        const val EXTRA_GAME = "com.example.chess_clock_app.extra.GAME"

        // Result buttons in the "Game details" popup and the PGN result each one stands for
        private val RESULT_BUTTONS = mapOf(
            "1-0" to R.id.resultWhiteWins,
            "1/2-1/2" to R.id.resultDraw,
            "0-1" to R.id.resultBlackWins,
            "*" to R.id.resultUnknown,
        )
    }

    private val viewModel: GameEditViewModel by viewModels { GameEditViewModel.Factory }

    // Only takes over the back button while there are unsaved changes
    private val unsavedChangesCallback = object : OnBackPressedCallback(false) {
        override fun handleOnBackPressed() {
            if (drawerLayout.isDrawerOpen(GravityCompat.START)) {
                drawerLayout.closeDrawer(GravityCompat.START)
                return
            }
            confirmLeave { finish() }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setActivityContent(R.layout.content_game_edit)

        if (intent.hasExtra(EXTRA_GAME)) {
            viewModel.loadGame(intent.getIntExtra(EXTRA_GAME, -1))
        } else {
            viewModel.startNewGame()
        }

        val titleText = findViewById<TextView>(R.id.gameTitle)
        val editGameButton = findViewById<ImageButton>(R.id.editGameButton)
        val deleteGameButton = findViewById<ImageButton>(R.id.deleteGameButton)
        val dateText = findViewById<TextView>(R.id.gameDate)
        val board = findViewById<ChessBoardView>(R.id.chessBoard)
        val whitePlayer = findViewById<TextView>(R.id.whitePlayer)
        val blackPlayer = findViewById<TextView>(R.id.blackPlayer)
        val movesRecyclerView = findViewById<RecyclerView>(R.id.movesRecyclerView)
        val noMovesText = findViewById<TextView>(R.id.noMovesText)
        val previousButton = findViewById<Button>(R.id.previousMoveButton)
        val saveButton = findViewById<Button>(R.id.saveMovesButton)
        val nextButton = findViewById<Button>(R.id.nextMoveButton)

        onBackPressedDispatcher.addCallback(this, unsavedChangesCallback)

        val moveAdapter = MoveAdapter()
        movesRecyclerView.layoutManager = LinearLayoutManager(this)
        movesRecyclerView.adapter = moveAdapter

        board.onMoveRequested = { from, to, isPromotion ->
            if (isPromotion) {
                askPromotionPiece { piece -> viewModel.makeMove(from, to, piece) }
            } else {
                viewModel.makeMove(from, to)
            }
        }

        editGameButton.setOnClickListener { showDetailsDialog() }
        deleteGameButton.setOnClickListener { confirmDelete() }

        // Tapping a player name opens a popup to edit it
        whitePlayer.setOnClickListener {
            showEditDialog(R.string.edit_player_name, viewModel.uiState.value.white, InputLimits.PLAYER_NAME) { newName ->
                viewModel.setPlayerName(Side.WHITE, newName)
            }
        }
        blackPlayer.setOnClickListener {
            showEditDialog(R.string.edit_player_name, viewModel.uiState.value.black, InputLimits.PLAYER_NAME) { newName ->
                viewModel.setPlayerName(Side.BLACK, newName)
            }
        }

        previousButton.setOnClickListener { viewModel.previousMove() }
        nextButton.setOnClickListener { viewModel.nextMove() }
        saveButton.setOnClickListener {
            viewModel.save()
            Toast.makeText(this, R.string.moves_saved, Toast.LENGTH_SHORT).show()
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.uiState.collect { state ->
                    if (state.notFound) {
                        Toast.makeText(this@GameEditActivity, R.string.game_not_found,
                            Toast.LENGTH_LONG).show()
                        finish()
                        return@collect
                    }
                    if (state.isLoading) return@collect

                    titleText.text = state.details.name
                    dateText.text = formatDateTime(state.details.createdAt)
                    whitePlayer.text = state.white
                    blackPlayer.text = state.black
                    board.position = state.position
                    board.lastMove = state.lastMove

                    moveAdapter.updateRows(state.moveRows)
                    noMovesText.visibility = if (state.moveRows.isEmpty()) View.VISIBLE else View.GONE
                    // Only scrolls if the current move is out of view
                    if (state.moveRows.isNotEmpty()) movesRecyclerView.scrollToPosition(state.currentRow)

                    previousButton.setEnabledLook(state.canGoBack)
                    nextButton.setEnabledLook(state.canGoForward)
                    saveButton.setEnabledLook(state.canSave)
                    unsavedChangesCallback.isEnabled = state.hasUnsavedChanges
                }
            }
        }
    }

    // Called by the back button (above) and by the drawer before it opens another screen
    override fun confirmLeave(proceed: () -> Unit) {
        if (!viewModel.uiState.value.hasUnsavedChanges) {
            proceed()
            return
        }
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.unsaved_changes_title)
            .setMessage(R.string.unsaved_changes_message)
            .setPositiveButton(R.string.save) { _, _ ->
                // Wait until the game is written before leaving
                lifecycleScope.launch {
                    viewModel.save().join()
                    proceed()
                }
            }
            .setNegativeButton(R.string.discard) { _, _ ->
                viewModel.discardChanges()
                proceed()
            }
            .setNeutralButton(R.string.cancel, null)
            .show()
    }

    // Popup with the PGN header values; the players are edited through the pills instead
    private fun showDetailsDialog() {
        val current = viewModel.uiState.value.details
        // Inflate with the popup's context, so the fields use the popup colors, not the screen's
        val builder = MaterialAlertDialogBuilder(this)
        val dialogView = LayoutInflater.from(builder.context).inflate(R.layout.dialog_game_details, null)
        val nameField = dialogView.findViewById<EditText>(R.id.detailsName)
        val siteField = dialogView.findViewById<EditText>(R.id.detailsSite)
        val dateField = dialogView.findViewById<EditText>(R.id.detailsDate)
        val roundField = dialogView.findViewById<EditText>(R.id.detailsRound)
        val resultGroup = dialogView.findViewById<MaterialButtonToggleGroup>(R.id.detailsResult)

        dialogView.findViewById<TextInputLayout>(R.id.detailsNameLayout).limitLength(InputLimits.GAME_NAME)
        dialogView.findViewById<TextInputLayout>(R.id.detailsSiteLayout).limitLength(InputLimits.SITE)
        dialogView.findViewById<TextInputLayout>(R.id.detailsRoundLayout).limitLength(InputLimits.ROUND)

        // "?" means unknown in PGN; shown as an empty field
        nameField.setText(current.name)
        siteField.setText(current.site.takeUnless { it == "?" })
        roundField.setText(current.round.takeUnless { it == "?" })
        resultGroup.check(RESULT_BUTTONS[current.result] ?: R.id.resultUnknown)

        // Only the day changes; the time of day of createdAt is kept
        var createdAt = current.createdAt
        dateField.setText(formatDate(toLocalDate(createdAt)))
        dateField.setOnClickListener {
            pickDate(toLocalDate(createdAt)) { picked ->
                createdAt = withDate(createdAt, picked)
                dateField.setText(formatDate(picked))
            }
        }

        builder
            .setTitle(R.string.game_details)
            .setView(dialogView)
            .setPositiveButton(R.string.ok) { _, _ ->
                val result = RESULT_BUTTONS.entries
                    .firstOrNull { it.value == resultGroup.checkedButtonId }?.key ?: "*"
                viewModel.updateDetails(
                    GameDetails(
                        name = nameField.text.toString(),
                        site = siteField.text.toString(),
                        createdAt = createdAt,
                        round = roundField.text.toString(),
                        result = result,
                    )
                )
            }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }

    // The date picker works with midnight UTC in milliseconds, so convert from and to LocalDate
    private fun pickDate(current: LocalDate, onPicked: (LocalDate) -> Unit) {
        val selection = current.atStartOfDay(ZoneOffset.UTC).toInstant().toEpochMilli()
        val picker = MaterialDatePicker.Builder.datePicker()
            .setTitleText(R.string.details_date)
            .setSelection(selection)
            .build()
        picker.addOnPositiveButtonClickListener { millis ->
            onPicked(Instant.ofEpochMilli(millis).atZone(ZoneOffset.UTC).toLocalDate())
        }
        picker.show(supportFragmentManager, "datePicker")
    }

    // Asks first; also for a new game that was never saved
    private fun confirmDelete() {
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.delete_game)
            .setMessage(getString(R.string.delete_game_message, viewModel.uiState.value.details.name))
            .setPositiveButton(R.string.delete) { _, _ ->
                // Wait until the game is deleted, then close the screen
                lifecycleScope.launch {
                    viewModel.deleteGame().join()
                    finish()
                }
            }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }

    private fun askPromotionPiece(onChosen: (PieceType) -> Unit) {
        val pieces = listOf(PieceType.QUEEN, PieceType.ROOK, PieceType.BISHOP, PieceType.KNIGHT)
        val names = arrayOf(
            getString(R.string.queen),
            getString(R.string.rook),
            getString(R.string.bishop),
            getString(R.string.knight),
        )
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.promote_to)
            .setItems(names) { _, which -> onChosen(pieces[which]) }
            .show()
    }
}
